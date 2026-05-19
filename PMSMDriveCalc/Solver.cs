using System;
using System.Collections.Generic;
using System.Numerics;

namespace PMSMDriveCalc
{
    public class Solver
    {
        public Solver(List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods)
        {
            VoltageU = voltageU;
            VoltageV = voltageV;
            VoltageW = voltageW;

            Time = time;
            this.periods = periods;

            VoltageUV = ListOperations.Minus(voltageU, voltageV);
            VoltageVW = ListOperations.Minus(voltageV, voltageW);
            VoltageWU = ListOperations.Minus(voltageW, voltageU);
        }

        public List<double> VoltageU { get; private set; }
        public List<double> VoltageV { get; private set; }
        public List<double> VoltageW { get; private set; }

        public List<double> VoltageUV { get; private set; }
        public List<double> VoltageVW { get; private set; }
        public List<double> VoltageWU { get; private set; }


        public List<double> CurrentU { get; protected set; }
        public List<double> CurrentV { get; protected set; }
        public List<double> CurrentW { get; protected set; }

        public List<double> CurrentFrequencies { get; protected set; }
        public List<double> CurrentAmplitudes { get; protected set; }
        public List<double> VoltageFrequencies { get; protected set; }
        public List<double> VoltageAmplitudes { get; protected set; }

        public List<double> LineToLineVoltageFrequencies { get; protected set; }
        public List<double> LineToLineVoltageAmplitudes { get; protected set; }
        public List<double> Time { get; private set; }
        protected int periods;

        /// <summary>
        /// Fundamental d-axis current (A) from synchronous demodulation —
        /// the DC-mean of the DQ-frame current over the steady-state window.
        /// Null for non-DQ solvers. Immune to switching-frequency spectral leakage.
        /// </summary>
        public double? IdFund { get; protected set; }

        /// <summary>
        /// Fundamental q-axis current (A) from synchronous demodulation —
        /// the DC-mean of the DQ-frame current over the steady-state window.
        /// Null for non-DQ solvers. Immune to switching-frequency spectral leakage.
        /// </summary>
        public double? IqFund { get; protected set; }

        /// <summary>
        /// Compute average electromagnetic torque (Nm) from solved abc-phase currents
        /// using Clarke + Park transformation into the dq synchronous frame.
        /// </summary>
        /// <param name="motor">PMSM dq-model (provides Ld, Lq, and PM flux linkage)</param>
        /// <returns>Average torque in Nm over the steady-state cycles</returns>
        public double CalculateTorque(PMSMdq motor)
        {
            if (CurrentU == null || CurrentU.Count == 0)
                throw new InvalidOperationException("Currents have not been solved yet.");

            int n = CurrentU.Count;
            double omega = 2.0 * Math.PI * motor.BaseFrequency;
            double sumTorque = 0.0;

            for (int i = 0; i < n; i++)
            {
                // Clarke transform: abc → αβ (zero-sequence-rejecting)
                double iAlpha = (2.0 * CurrentU[i] - CurrentV[i] - CurrentW[i]) / 3.0;
                double iBeta  = (CurrentV[i] - CurrentW[i]) / Math.Sqrt(3.0);

                // Park transform: αβ → dq (rotor angle = ω*t)
                double theta = omega * Time[i];
                double cosT = Math.Cos(theta);
                double sinT = Math.Sin(theta);
                double id = iAlpha * cosT + iBeta * sinT;
                double iq = -iAlpha * sinT + iBeta * cosT;

                sumTorque += motor.CalculateTorque(id, iq);
            }

            return sumTorque / n;
        }
    }

    

    public class ACSolver : Solver
    {
        public ACSolver(PMSMWithVariableInductance motorDataWithVariableInductance,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods) :
            base(voltageU, voltageV, voltageW, time, periods)
        {
            this.MotorDataWithVariableInductance = motorDataWithVariableInductance;
            this.MotorDataWithConstantInductance = null;
        }

        public ACSolver(PMSMWithConstantInductance motorDataWithConstantInductance,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods) :
            base(voltageU, voltageV, voltageW, time, periods)
        {
            this.MotorDataWithVariableInductance = null;
            this.MotorDataWithConstantInductance = motorDataWithConstantInductance;
        }

        public PMSMWithVariableInductance MotorDataWithVariableInductance { get; private set; }
        public PMSMWithConstantInductance MotorDataWithConstantInductance { get; private set; }


    }


    /// <summary>
    /// [Obsolete] Frequency-domain PMSM current solver.
    /// Replaced by DQTransientSolverStar with steady-state operating-point initialization.
    /// Kept for backward compatibility; all dispatches now route to the DQ transient solver.
    /// </summary>
    [Obsolete("Replaced by DQTransientSolverStar with steady-state ICs. AC solver dispatches are redirected to DQTransientSolverStar in DQDriveCalculator.Compute().", false)]
    public class ACSolverStar : ACSolver
    {
        /// <summary>Maximum number of harmonics used for time-domain reconstruction.</summary>
        private const int MAX_RECONSTRUCT_HARMONICS = 200;

        public ACSolverStar(PMSMWithVariableInductance motorDataWithVariableInductance,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods, double pwmFrequency) :
            base(motorDataWithVariableInductance, voltageU, voltageV, voltageW, time, periods)
        {
            _getCurrents(periods, pwmFrequency);
        }

        public ACSolverStar(PMSMWithConstantInductance motorDataWithConstantInductance,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods, double pwmFrequency) :
            base(motorDataWithConstantInductance, voltageU, voltageV, voltageW, time, periods)
        {
            _getCurrents(periods, pwmFrequency);
        }

        // ─────────────────────────────────────────────────────────────────────
        // Internal data structures
        // ─────────────────────────────────────────────────────────────────────
        private struct HarmonicPhasor
        {
            public double FrequencyHz;
            public double Order;
            /// <summary>Complex amplitude for phase U (A).</summary>
            public Complex Iu;
            /// <summary>Complex amplitude for phase V (A).</summary>
            public Complex Iv;
            /// <summary>Complex amplitude for phase W (A).</summary>
            public Complex Iw;
        }

        // ─────────────────────────────────────────────────────────────────────
        // Main solver entry point (called from constructor)
        // ─────────────────────────────────────────────────────────────────────
        private void _getCurrents(int periods, double pwmFrequency)
        {
            // ── Step 1: FFT of line-to-line voltages ──
            FFTContainer fftuv = FFTOperations.GetFFT(this.VoltageUV.ToArray());
            FFTContainer fftvw = FFTOperations.GetFFT(this.VoltageVW.ToArray());
            FFTContainer fftwu = FFTOperations.GetFFT(this.VoltageWU.ToArray());

            // ── Step 2: Extract motor parameters ──
            bool isConstantL = (this.MotorDataWithConstantInductance != null);
            double f1, R, psiPM;
            if (isConstantL)
            {
                var m = this.MotorDataWithConstantInductance;
                f1 = m.BaseFrequency;  R = m.PhaseResistance;  psiPM = m.PMFluxLinkage;
            }
            else
            {
                var m = this.MotorDataWithVariableInductance;
                f1 = m.BaseFrequency;  R = m.PhaseResistance;  psiPM = m.PMFluxLinkage;
            }
            double omega1 = 2.0 * Math.PI * f1;   // fundamental electrical angular speed

            // ── Step 3: Precompute back-EMF line-to-line phasors (fundamental only) ──
            // e_a = ω₁·ψ_pm·sin(ω₁t),  e_b = e_a shifted -120°,  e_c = e_a shifted -240°
            // Line-to-line in phasor form (peak amplitude):
            //   E_uv = j·ω₁·ψ_pm·(1 - e^{-j·2π/3}) = ω₁·ψ_pm·(-√3/2 + j·3/2)
            //   E_vw = j·ω₁·ψ_pm·(e^{-j·2π/3} - e^{-j·4π/3}) = ω₁·ψ_pm·√3
            double Ebase = omega1 * psiPM;
            double sqrt3 = Math.Sqrt(3.0);
            Complex E_uv = new Complex(Ebase * sqrt3 * 0.5, -Ebase * 1.5);
            Complex E_vw = new Complex(-Ebase * sqrt3, 0.0);

            // ── Step 4: Build harmonic list sorted by amplitude ──
            int numBins = fftuv.Amplitude.Count;
            var harmonics = new List<(double amp, double phaseDeg, int binIdx, double order)>();
            for (int i = 0; i < numBins; i++)
            {
                double avgAmp = (fftuv.Amplitude[i] + fftvw.Amplitude[i] + fftwu.Amplitude[i]) / 3.0;
                double realOrder = (double)i / (double)periods;
                if (Math.Abs(realOrder) > 1e-3 && avgAmp > 0)
                    harmonics.Add((avgAmp, fftuv.Phase[i], i, realOrder));
            }
            // Sort descending by amplitude so small harmonics can be truncated if needed
            harmonics.Sort((a, b) => b.amp.CompareTo(a.amp));

            // ── Step 5: Compute harmonic currents using simplified impedance ──
            // For a balanced Y-connected 3-phase load with self-Ls and mutual-Lm:
            //   After eliminating the neutral constraint iu+iv+iw=0, the differential-mode
            //   impedance is Z_eq = R + jω(Ls - Lm).  The line-to-line admittance is 1/(3·Z_eq).
            //
            // Closed-form currents from line-to-line voltages:
            //   iu = (u_vw + 2·u_uv) / (3·Z_eq)
            //   iv = (u_vw - u_uv) / (3·Z_eq)
            //   iw = -(iu + iv)
            //
            // For the fundamental, subtract back-EMF line-to-line voltages from the applied voltages.

            int harmCount = harmonics.Count;
            var phasors = new List<HarmonicPhasor>(harmCount);
            double inv3 = 1.0 / 3.0;

            for (int h = 0; h < harmCount; h++)
            {
                var (amp, phaseDeg, binIdx, order) = harmonics[h];
                double fHarm = order * f1;
                double omega = 2.0 * Math.PI * fHarm;
                bool isFund = Math.Abs(order - 1.0) < 1e-3;

                // ── Impedance Z_eq = R + jω(Ls - Lm) ──
                Complex Zeq;
                if (isConstantL)
                {
                    double Ldm = this.MotorDataWithConstantInductance.SelfInductance
                               - this.MotorDataWithConstantInductance.MutualInductance;
                    Zeq = new Complex(R, omega * Ldm);
                }
                else
                {
                    var ind = this.MotorDataWithVariableInductance.GetInductanceAtFrequency(fHarm);
                    double Ldm = ind[0] - ind[1];
                    Zeq = new Complex(R, omega * Ldm);
                }

                Complex invDenom = inv3 / Zeq;  // 1 / (3·Z_eq)

                // ── Line-to-line voltage phasors (peak) ──
                // FFT gives phase in degrees for cos reference.
                // u_uv(t) = amp·cos(2π·f·t + φ_fft)  (peak, not RMS)
                // Phasor: U_uv = amp·e^{j·φ_fft}
                //
                // But note: the FFT convention uses cos reference, and the original code
                // added 180° when constructing u_uv.  We replicate that here.
                double phiRad = (phaseDeg + 180.0) * Math.PI / 180.0;
                Complex u_uv = Complex.FromPolarCoordinates(amp, phiRad);
                Complex u_vw = Complex.FromPolarCoordinates(amp, phiRad - 2.0 * Math.PI / 3.0);

                // Subtract back-EMF for fundamental
                if (isFund)
                {
                    u_uv -= E_uv;
                    u_vw -= E_vw;
                }

                // ── Closed-form currents ──
                Complex iu = (u_vw + 2.0 * u_uv) * invDenom;
                Complex iv = (u_vw - u_uv) * invDenom;
                Complex iw = -iu - iv;

                phasors.Add(new HarmonicPhasor
                {
                    FrequencyHz = fHarm,
                    Order = order,
                    Iu = iu,
                    Iv = iv,
                    Iw = iw,
                });
            }

            // Sort by frequency (ascending) for cleaner output
            phasors.Sort((a, b) => a.FrequencyHz.CompareTo(b.FrequencyHz));

            // ── Store frequency-domain results ──
            this.CurrentFrequencies = new List<double>(harmCount);
            this.CurrentAmplitudes = new List<double>(harmCount);
            for (int i = 0; i < harmCount; i++)
            {
                this.CurrentFrequencies.Add(phasors[i].FrequencyHz);
                this.CurrentAmplitudes.Add(phasors[i].Iu.Magnitude);
            }

            // ── Step 6: Time-domain waveform reconstruction via trigonometric recurrence ──
            //
            // Instead of calling Math.Cos(ωt+φ) for every harmonic × time-point,
            // we use the recurrence:
            //   cₖ₊₁ = cₖ·cos(ωΔt) - sₖ·sin(ωΔt)
            //   sₖ₊₁ = sₖ·cos(ωΔt) + cₖ·sin(ωΔt)
            // where cₖ = cos(ω·tₖ + φ), sₖ = sin(ω·tₖ + φ).
            // Initial: c₀ = cos(φ), s₀ = sin(φ).
            //
            // For the accumulated waveform at each time-point we only need cₖ (the cosine),
            // but the recurrence requires both c and s.
            //
            // We accumulate the top MAX_RECONSTRUCT_HARMONICS (sorted by amplitude).

            int nTime = this.Time.Count;
            double dt = nTime > 1 ? this.Time[1] - this.Time[0] : 0.0;

            double[] iuWave = new double[nTime];
            double[] ivWave = new double[nTime];
            double[] iwWave = new double[nTime];

            // Build amplitude-sorted index list for selecting top harmonics
            int[] sortedByAmp = new int[harmCount];
            for (int i = 0; i < harmCount; i++) sortedByAmp[i] = i;
            Array.Sort(sortedByAmp, (a, b) => phasors[b].Iu.Magnitude.CompareTo(phasors[a].Iu.Magnitude));

            int nReconstruct = Math.Min(MAX_RECONSTRUCT_HARMONICS, harmCount);

            for (int hi = 0; hi < nReconstruct; hi++)
            {
                int idx = sortedByAmp[hi];
                var hp = phasors[idx];
                double omegaH = 2.0 * Math.PI * hp.FrequencyHz;

                // Initialise recurrence
                double cos_dt = Math.Cos(omegaH * dt);
                double sin_dt = Math.Sin(omegaH * dt);

                // Phase-U recurrence state
                double cu = hp.Iu.Real;   // |Iu|·cos(φ)  at t=0
                double su = hp.Iu.Imaginary; // |Iu|·sin(φ)  at t=0
                // Phase-V
                double cv = hp.Iv.Real;
                double sv = hp.Iv.Imaginary;
                // Phase-W
                double cw = hp.Iw.Real;
                double sw = hp.Iw.Imaginary;

                for (int t = 0; t < nTime; t++)
                {
                    iuWave[t] += cu;
                    ivWave[t] += cv;
                    iwWave[t] += cw;

                    // Rotate by ω·dt
                    double cu_new = cu * cos_dt - su * sin_dt;
                    double su_new = su * cos_dt + cu * sin_dt;
                    double cv_new = cv * cos_dt - sv * sin_dt;
                    double sv_new = sv * cos_dt + cv * sin_dt;
                    double cw_new = cw * cos_dt - sw * sin_dt;
                    double sw_new = sw * cos_dt + cw * sin_dt;

                    cu = cu_new; su = su_new;
                    cv = cv_new; sv = sv_new;
                    cw = cw_new; sw = sw_new;
                }
            }

            this.CurrentU = new List<double>(iuWave);
            this.CurrentV = new List<double>(ivWave);
            this.CurrentW = new List<double>(iwWave);

            // ── Step 7: Output FFT spectra for voltages (for convenience) ──
            {
                FFTContainer fftVoltageU = FFTOperations.GetFFT(this.VoltageU.ToArray());
                this.VoltageAmplitudes = new List<double> { };
                this.VoltageFrequencies = new List<double> { };
                for (int i = 0; i < fftVoltageU.Amplitude.Count; i++)
                {
                    int orderNow = fftVoltageU.Order[i];
                    double realOrder = orderNow / (double)this.periods;
                    this.VoltageAmplitudes.Add(fftVoltageU.Amplitude[i]);
                    this.VoltageFrequencies.Add(realOrder * f1);
                }
            }
            {
                FFTContainer fftVoltageUV = FFTOperations.GetFFT(this.VoltageUV.ToArray());
                this.LineToLineVoltageAmplitudes = new List<double> { };
                this.LineToLineVoltageFrequencies = new List<double> { };
                for (int i = 0; i < fftVoltageUV.Amplitude.Count; i++)
                {
                    int orderNow = fftVoltageUV.Order[i];
                    double realOrder = orderNow / (double)this.periods;
                    this.LineToLineVoltageAmplitudes.Add(fftVoltageUV.Amplitude[i]);
                    this.LineToLineVoltageFrequencies.Add(realOrder * f1);
                }
            }
        }
    }



    [Obsolete("Replaced by DQTransientSolverWithLCFilterSemi and DQTransientSolverWithLCFilterFull. AC solver dispatches are redirected in DQDriveCalculator.Compute().", false)]
    public class ACSolverStarWithOutputFilter : ACSolver
    {
        public ACSolverStarWithOutputFilter(PMSMWithVariableInductance motorDataWithVariableInductance,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods, double pwmFrequency) :
            base(motorDataWithVariableInductance, voltageU, voltageV, voltageW, time, periods)
        {
            _getCurrents(periods, pwmFrequency);
        }

        public ACSolverStarWithOutputFilter(PMSMWithConstantInductance motorDataWithConstantInductance,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods, double pwmFrequency) :
            base(motorDataWithConstantInductance, voltageU, voltageV, voltageW, time, periods)
        {
            _getCurrents(periods, pwmFrequency);
        }

        private void _getCurrents(int periods, double pwmFrequency)
        {
            FFTContainer fftuv = FFTOperations.GetFFT(this.VoltageUV.ToArray());
            FFTContainer fftvw = FFTOperations.GetFFT(this.VoltageVW.ToArray());
            FFTContainer fftwu = FFTOperations.GetFFT(this.VoltageWU.ToArray());

            double fundamentalFrequency = (this.MotorDataWithConstantInductance == null) ?
                this.MotorDataWithVariableInductance.BaseFrequency :
                this.MotorDataWithConstantInductance.BaseFrequency;

            //double fstep = fundamentalFrequency / (double)periods;
            //double fmax = 5.0 * pwmFrequency;
            //int num5times = (int)Math.Round(fmax / fstep);


            int num = fftuv.Amplitude.Count;

            double[] meanAmp = new double[num];
            int[] order = new int[num];
            double[] phase = new double[num];
            for (int i = 0; i < num; i++)
            {
                meanAmp[i] = (fftuv.Amplitude[i] + fftvw.Amplitude[i] + fftwu.Amplitude[i]) / 3.0;
                order[i] = i;
                phase[i] = fftuv.Phase[i];
            }

            Array.Sort(meanAmp, order);
            Array.Reverse(meanAmp);
            Array.Reverse(order);

            List<double> _sortedAmp = new List<double> { };
            List<double> _sortedOrd = new List<double> { };
            List<double> _sortedPh = new List<double> { };
            for (int i = 0; i < num; i++)
            {
                int _orderNow = order[i];
                double _realOrder = _orderNow / (double)this.periods;
                if (Math.Abs(_realOrder) > 1e-3)
                {
                    _sortedAmp.Add(meanAmp[i]);
                    _sortedOrd.Add(_realOrder);
                    _sortedPh.Add(phase[order[i]]);
                }
            }

            int len = _sortedAmp.Count;
            List<Complex> _currentU = new List<Complex> { };
            List<Complex> _currentV = new List<Complex> { };
            List<Complex> _currentW = new List<Complex> { };
            List<double> _freq_iu = new List<double> { };


            for (int i = 0; i < len; i++)
            {
                var currents = _getHarmonicCurrent(_sortedAmp[i], _sortedPh[i], _sortedOrd[i]);
                _currentU.Add(currents[0]);
                _currentV.Add(currents[1]);
                _currentW.Add(currents[2]);
                _freq_iu.Add(_sortedOrd[i] * fundamentalFrequency);
            }

            var _currentUArray = _currentU.ToArray();
            var _currentFrequenciesArray = _freq_iu.ToArray();
            Array.Sort(_currentFrequenciesArray, _currentUArray);

            this.CurrentFrequencies = new List<double>(_currentFrequenciesArray);

            this.CurrentAmplitudes = new List<double> { };
            for (int i = 0; i < len; i++)
            {
                this.CurrentAmplitudes.Add(_currentUArray[i].Magnitude);
            }

            double[] _ampU = new double[_currentU.Count];
            int[] _indexU = new int[_currentU.Count];
            for (int i = 0; i < _currentU.Count; i++)
            {
                _ampU[i] = _currentU[i].Magnitude;
                _indexU[i] = i;
            }
            Array.Sort(_ampU, _indexU);
            Array.Reverse(_indexU);
            List<double> _iu_list = new List<double> { };
            List<double> _iv_list = new List<double> { };
            List<double> _iw_list = new List<double> { };
            foreach (double _t in this.Time)
            {
                double _iu_now = 0.0;
                double _iv_now = 0.0;
                double _iw_now = 0.0;
                for (int i = 0; i < 50; i++)
                {
                    int ind = _indexU[i];
                    _iu_now += _currentU[ind].Magnitude * Math.Cos(_freq_iu[ind] *
                        2.0 * Math.PI * _t + _currentU[ind].Phase);
                    _iv_now += _currentV[ind].Magnitude * Math.Cos(_freq_iu[ind] *
                        2.0 * Math.PI * _t + _currentV[ind].Phase);
                    _iw_now += _currentW[ind].Magnitude * Math.Cos(_freq_iu[ind] *
                        2.0 * Math.PI * _t + _currentW[ind].Phase);
                }
                _iu_list.Add(_iu_now);
                _iv_list.Add(_iv_now);
                _iw_list.Add(_iw_now);
            }
            this.CurrentU = _iu_list;
            this.CurrentV = _iv_list;
            this.CurrentW = _iw_list;


            FFTContainer fftVoltageU = FFTOperations.GetFFT(this.VoltageU.ToArray());
            this.VoltageAmplitudes = new List<double> { };
            this.VoltageFrequencies = new List<double> { };
            for (int i = 0; i < fftVoltageU.Amplitude.Count; i++)
            {
                int _orderNow = fftVoltageU.Order[i];
                double _realOrder = _orderNow / (double)this.periods;
                this.VoltageAmplitudes.Add(fftVoltageU.Amplitude[i]);
                this.VoltageFrequencies.Add(_realOrder * fundamentalFrequency);
            }


            FFTContainer fftVoltageUV = FFTOperations.GetFFT(this.VoltageUV.ToArray());
            this.LineToLineVoltageAmplitudes = new List<double> { };
            this.LineToLineVoltageFrequencies = new List<double> { };
            for (int i = 0; i < fftVoltageUV.Amplitude.Count; i++)
            {
                int _orderNow = fftVoltageUV.Order[i];
                double _realOrder = _orderNow / (double)this.periods;
                this.LineToLineVoltageAmplitudes.Add(fftVoltageUV.Amplitude[i]);
                this.LineToLineVoltageFrequencies.Add(_realOrder * fundamentalFrequency);
            }

        }


        private List<Complex> _getHarmonicCurrent(double amplitude, double phase, double order)
        {
            double fundamentalFrequency = (this.MotorDataWithConstantInductance == null) ?
                this.MotorDataWithVariableInductance.BaseFrequency :
                this.MotorDataWithConstantInductance.BaseFrequency;

            double omega = 2.0 * Math.PI * order * fundamentalFrequency;
            Complex jomega = new Complex(0.0, omega);
            double factor = (Math.Abs(order - 1) < 1e-3) ? 1.0 : 0.0;

            double phaseResistance = (this.MotorDataWithConstantInductance == null) ?
                this.MotorDataWithVariableInductance.PhaseResistance :
                this.MotorDataWithConstantInductance.PhaseResistance;


            double pMFluxLinkage = (this.MotorDataWithConstantInductance == null) ?
                this.MotorDataWithVariableInductance.PMFluxLinkage :
                this.MotorDataWithConstantInductance.PMFluxLinkage;


            double selfInductance, mutualInductance;
            if (this.MotorDataWithConstantInductance == null)
            {
                var _ind = this.MotorDataWithVariableInductance.GetInductanceAtFrequency(order * fundamentalFrequency);
                selfInductance = _ind[0];
                mutualInductance = _ind[1];
            }
            else
            {
                selfInductance = this.MotorDataWithConstantInductance.SelfInductance;
                mutualInductance = this.MotorDataWithConstantInductance.MutualInductance;
            }



            Complex a = phaseResistance + jomega * selfInductance;
            Complex b = jomega * mutualInductance;
            Complex c = jomega * mutualInductance;
            Complex d = new Complex(0.0, -fundamentalFrequency * 2.0 * Math.PI * pMFluxLinkage) * factor;

            Complex e = jomega * mutualInductance;
            Complex f = phaseResistance + jomega * selfInductance;
            Complex g = jomega * mutualInductance;
            Complex h = new Complex(0.0, -fundamentalFrequency * 2.0 * Math.PI * pMFluxLinkage) * factor *
                Complex.Pow(Math.E, new Complex(0.0, -2.0 / 3.0 * Math.PI));

            Complex i = jomega * mutualInductance;
            Complex j = jomega * mutualInductance;
            Complex k = phaseResistance + jomega * selfInductance;
            Complex l = new Complex(0.0, -fundamentalFrequency * 2.0 * Math.PI * pMFluxLinkage) * factor *
                Complex.Pow(Math.E, new Complex(0.0, -4.0 / 3.0 * Math.PI));

            Complex A = a - e;
            Complex B = b - f;
            Complex C = c - g;
            Complex D = d - h;

            Complex E = e - i;
            Complex F = f - j;
            Complex G = g - k;
            Complex H = h - l;

            Complex u_uv = amplitude * Complex.Pow(Math.E, new Complex(0.0, (phase + 180.0) / 180.0 * Math.PI));
            Complex u_vw = u_uv * Complex.Pow(Math.E, new Complex(0.0, -2.0 / 3.0 * Math.PI));
            Complex u_wu = u_uv * Complex.Pow(Math.E, new Complex(0.0, -4.0 / 3.0 * Math.PI));

            Complex iu = (B * H - B * u_vw - C * H + C * u_vw - D * F + D * G + F * u_uv - G * u_uv) /
                (A * F - A * G - B * E + B * G + C * E - C * F);

            Complex iv = (-A * H + A * u_vw + C * H - C * u_vw + D * E - D * G - E * u_uv + G * u_uv) /
                (A * F - A * G - B * E + B * G + C * E - C * F);

            Complex iw = -iu - iv;

            return new List<Complex> { iu, iv, iw };
        }


    }




    public class TransientSolver : Solver
    {
        public PMSMWithConstantInductance MotorData { get; private set; }



        public TransientSolver(PMSMWithConstantInductance motorData,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods) :
            base(voltageU, voltageV, voltageW, time, periods)
        {
            this.MotorData = motorData;
        }

        public TransientSolver(PMSMWithVariableInductance motorData,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods) :
            base(voltageU, voltageV, voltageW, time, periods)
        {
            PMSMWithConstantInductance _motorData = new PMSMWithConstantInductance(
                motorData.Poles, motorData.PMFluxLinkage,
                motorData.SelfInductance[0],
                motorData.MutualInductance[0],
                motorData.PhaseResistance);
            _motorData.SpeedRPM = motorData.SpeedRPM;
            this.MotorData = _motorData;
        }
    }

    public class TransientSolverStar : TransientSolver
    {
        public TransientSolverStar(PMSMWithConstantInductance motorData,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods) :
            base(motorData, voltageU, voltageV, voltageW, time, periods)
        {
            _getCurrents();
        }

        public TransientSolverStar(PMSMWithVariableInductance motorData,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods) :
            base(motorData, voltageU, voltageV, voltageW, time, periods)
        {
            _getCurrents();
        }

        private void _getCurrents()
        {
            double R = MotorData.PhaseResistance;
            double L = MotorData.SelfInductance;
            double M = MotorData.MutualInductance;
            double E_amp = MotorData.AngularSpeed * MotorData.PMFluxLinkage;
            double omega = MotorData.AngularSpeed;

            int len = this.Time.Count;
            double dt = Time[1] - Time[0];

            // ── Precompute loop invariants ──
            // After differencing phases + constraint iu+iv+iw=0:
            // system reduces to   α·(iu-iv) = 2·u_uv - u_vw + history_terms
            // with α = R + (L-M)/Δt, denominator = 3·α
            double Lm   = (L - M) / dt;        // (L-M)/Δt
            double alpha = R + Lm;              // R + (L-M)/Δt
            double Dden  = 3.0 * alpha;         // common denominator for iu, iv

            // ── Angle recurrence: sin/cos rotation per step (avoids Math.Sin) ──
            double sin_a = 0.0;                 // sin(θ) at t=0 (first step uses t=dt)
            double cos_a = 1.0;                 // cos(θ) at t=0
            double cos_dt = Math.Cos(omega * dt);
            double sin_dt = Math.Sin(omega * dt);

            // Phase-shift constants for back-EMF (120° / 240° offsets)
            const double cos120 = -0.5;
            const double sin120 = 0.8660254037844386;  // √3/2

            // ── Preallocate lists ──
            CurrentU = new List<double>(len) { 0.0 };
            CurrentV = new List<double>(len) { 0.0 };
            CurrentW = new List<double>(len) { 0.0 };

            // ── State variables ──
            double iu = 0.0, iv = 0.0, iw = 0.0;
            double di_uv_prev = 0.0;   // iu_prev - iv_prev
            double di_vw_prev = 0.0;   // iv_prev - iw_prev

            for (int index = 1; index < len; index++)
            {
                // ── Advance angle θ = ω·tₖ via rotation recurrence ──
                double sn = sin_a * cos_dt + cos_a * sin_dt;
                double cs = cos_a * cos_dt - sin_a * sin_dt;
                sin_a = sn;
                cos_a = cs;

                // ── Back-EMF (3-phase from sinθ, cosθ without extra Sin calls) ──
                double eu = E_amp * sn;
                double ev = E_amp * (sn * cos120 - cs * sin120);
                double ew = E_amp * (sn * cos120 + cs * sin120);

                // ── History + back-EMF differences (D=d-h, H=h-l) ──
                double D = -Lm * di_uv_prev + (eu - ev);
                double H = -Lm * di_vw_prev + (ev - ew);

                // ── Line-to-line voltages ──
                double u_uv = this.VoltageUV[index];
                double u_vw = this.VoltageVW[index];

                // ── Closed-form solution (simplified from 3×3 matrix) ──
                iu = (-H + u_vw - 2.0 * D + 2.0 * u_uv) / Dden;
                iv = (-H + u_vw + D - u_uv) / Dden;
                iw = -iu - iv;

                CurrentU.Add(iu);
                CurrentV.Add(iv);
                CurrentW.Add(iw);

                // ── Update history differences for next step ──
                di_uv_prev = iu - iv;
                di_vw_prev = iv - iw;
            }

            // select last 10 periods for the FFT
            double period = 1.0 / MotorData.BaseFrequency;
            List<double> _iu = new List<double> { };
            List<double> _uu = new List<double> { };
            List<double> _uuv = new List<double> { };
            double _t_count = 0.0;
            for (int i = CurrentU.Count - 1; i > 0; i--)
            {
                if (_t_count >= 10.0 * period) break;
                _iu.Add(CurrentU[i]);
                _uu.Add(VoltageU[i]);
                _uuv.Add(VoltageUV[i]);
                _t_count += Time[1] - Time[0];
            }

            double[] _iu_array = _iu.ToArray();
            double[] _uu_array = _uu.ToArray();
            double[] _uuv_array = _uuv.ToArray();
            Array.Reverse(_iu_array);
            Array.Reverse(_uu_array);
            Array.Reverse(_uuv_array);

            FFTContainer _fft_iu = FFTOperations.GetFFT(_iu_array);
            FFTContainer _fft_uu = FFTOperations.GetFFT(_uu_array);
            FFTContainer _fft_uuv = FFTOperations.GetFFT(_uuv_array);

            List<double> _amp_iu = new List<double> { };
            List<double> _freq_iu = new List<double> { };
            for (int i = 0; i < _fft_iu.Amplitude.Count; i++)
            {
                int _order = _fft_iu.Order[i];
                double _actual_order = _order / 10.0;
                _amp_iu.Add(_fft_iu.Amplitude[i]);
                _freq_iu.Add(_actual_order * MotorData.BaseFrequency);
            }
            CurrentAmplitudes = _amp_iu;
            CurrentFrequencies = _freq_iu;

            List<double> _amp_uu = new List<double> { };
            List<double> _freq_uu = new List<double> { };
            for (int i = 0; i < _fft_uu.Amplitude.Count; i++)
            {
                int _order = _fft_uu.Order[i];
                double _actual_order = _order / 10.0;
                _amp_uu.Add(_fft_uu.Amplitude[i]);
                _freq_uu.Add(_actual_order * MotorData.BaseFrequency);
            }
            VoltageAmplitudes = _amp_uu;
            VoltageFrequencies = _freq_uu;

            List<double> _amp_uuv = new List<double> { };
            List<double> _freq_uuv = new List<double> { };
            for (int i = 0; i < _fft_uuv.Amplitude.Count; i++)
            {
                int _order = _fft_uuv.Order[i];
                double _actual_order = _order / 10.0;
                _amp_uuv.Add(_fft_uuv.Amplitude[i]);
                _freq_uuv.Add(_actual_order * MotorData.BaseFrequency);
            }
            LineToLineVoltageAmplitudes = _amp_uuv;
            LineToLineVoltageFrequencies = _freq_uuv;


        }
    }

    /// <summary>
    /// Coupled LC-filter + PMSM transient solver.
    /// Integrates the filter inductor, filter capacitor, and motor as a single
    /// 6-state ODE system (two decoupled 3×3 blocks for U/V phases) using
    /// Backward Euler with closed-form per-step solution via Cramer's rule.
    /// </summary>
    public class TransientSolverWithLCFilter : TransientSolver
    {
        private double Lf;
        private double Cf;

        // ── New outputs: motor terminal voltages (= filter capacitor voltages) ──
        public List<double> MotorVoltageU { get; private set; }
        public List<double> MotorVoltageV { get; private set; }
        public List<double> MotorVoltageW { get; private set; }

        // ── New outputs: filter inductor currents ──
        public List<double> FilterInductorCurrentU { get; private set; }
        public List<double> FilterInductorCurrentV { get; private set; }
        public List<double> FilterInductorCurrentW { get; private set; }

        /// <summary>
        /// Create a coupled LC-filter + motor transient solver
        /// with constant motor inductance.
        /// </summary>
        /// <param name="motorData">Motor with constant inductance</param>
        /// <param name="voltageU">Raw PWM phase-U voltage (V)</param>
        /// <param name="voltageV">Raw PWM phase-V voltage (V)</param>
        /// <param name="voltageW">Raw PWM phase-W voltage (V)</param>
        /// <param name="time">Time vector (s)</param>
        /// <param name="periods">Number of fundamental periods</param>
        /// <param name="lf">Filter inductance per phase (H)</param>
        /// <param name="cf">Filter capacitance per phase (F), Y-connected</param>
        public TransientSolverWithLCFilter(PMSMWithConstantInductance motorData,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods,
            double lf, double cf)
            : base(motorData, voltageU, voltageV, voltageW, time, periods)
        {
            if (lf <= 0)
                throw new ArgumentException("Filter inductance must be positive.", nameof(lf));
            if (cf <= 0)
                throw new ArgumentException("Filter capacitance must be positive.", nameof(cf));
            Lf = lf;
            Cf = cf;
            _getCurrents();
        }

        /// <summary>
        /// Create a coupled LC-filter + motor transient solver
        /// from a variable-inductance motor model.
        /// The variable inductance is collapsed to its fundamental-frequency value.
        /// </summary>
        public TransientSolverWithLCFilter(PMSMWithVariableInductance motorData,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods,
            double lf, double cf)
            : base(motorData, voltageU, voltageV, voltageW, time, periods)
        {
            if (lf <= 0)
                throw new ArgumentException("Filter inductance must be positive.", nameof(lf));
            if (cf <= 0)
                throw new ArgumentException("Filter capacitance must be positive.", nameof(cf));
            Lf = lf;
            Cf = cf;
            _getCurrents();
        }

        private void _getCurrents()
        {
            // ── Extract motor parameters ──
            double R = MotorData.PhaseResistance;
            double L = MotorData.SelfInductance;
            double M = MotorData.MutualInductance;
            double Leq = L - M;                               // equivalent differential-mode inductance
            double E_amp = MotorData.AngularSpeed * MotorData.PMFluxLinkage;
            double omega = MotorData.AngularSpeed;

            int len = Time.Count;
            if (len < 2)
                throw new InvalidOperationException("Time vector must contain at least 2 points.");
            double dt = Time[1] - Time[0];

            // ── Precompute time-invariant constants (design doc § 4.1) ──
            double a = dt / Lf;                               // Δt / Lf
            double b = dt / Cf;                               // Δt / Cf
            double c = dt / Leq;                              // Δt / Leq
            double alpha = 1.0 + c * R;                       // motor diagonal coefficient
            double K_ab = 1.0 + a * b;                        // 1 + a·b
            double Dinv = 1.0 / (alpha * K_ab + b * c);       // 1 / determinant (precompute inverse)

            // ── Angle recurrence for back-EMF (avoids per-step Math.Sin/Math.Cos) ──
            double sin_a = 0.0;                               // sin(θ) at t=0
            double cos_a = 1.0;                               // cos(θ) at t=0
            double cos_dt = Math.Cos(omega * dt);
            double sin_dt = Math.Sin(omega * dt);

            // Phase-shift constants (120° / 240° offsets)
            const double cos120 = -0.5;
            const double sin120 = 0.8660254037844386;         // √3/2

            // ── Initialize state arrays (index → time step k) ──
            double[] i_Lf_u = new double[len];
            double[] i_Lf_v = new double[len];
            double[] v_Cf_u = new double[len];
            double[] v_Cf_v = new double[len];
            double[] i_m_u = new double[len];
            double[] i_m_v = new double[len];
            // All initial values are 0.0 (default for double arrays)

            // ── Main time-stepping loop (design doc § 5.2 sequential computation) ──
            for (int k = 1; k < len; k++)
            {
                // ── Advance rotor angle θ = ω·tₖ via trigonometric recurrence ──
                double sn = sin_a * cos_dt + cos_a * sin_dt;
                double cs = cos_a * cos_dt - sin_a * sin_dt;
                sin_a = sn;
                cos_a = cs;

                // ── Back-EMF for all three phases (from sinθ, cosθ) ──
                double eu = E_amp * sn;
                double ev = E_amp * (sn * cos120 - cs * sin120);
                double ew = E_amp * (sn * cos120 + cs * sin120);

                // ═══════════════════════════════════════════════════════
                // U-phase solve (3×3 block, design doc eq. (7) + sequential)
                // ═══════════════════════════════════════════════════════
                double B1_u = i_Lf_u[k - 1] + a * VoltageU[k];
                double B2_u = v_Cf_u[k - 1];
                double B3_u = i_m_u[k - 1] - c * eu;
                double B2pbB1_u = B2_u + b * B1_u;

                // Step 1: motor current (closed-form, design doc eq. 7)
                i_m_u[k] = (B3_u * K_ab + c * B2pbB1_u) * Dinv;

                // Step 2: capacitor voltage from motor Backward Euler equation
                v_Cf_u[k] = (Leq / dt) * (alpha * i_m_u[k] - i_m_u[k - 1]) + eu;

                // Step 3: inductor current from capacitor Backward Euler equation
                i_Lf_u[k] = i_m_u[k] + (Cf / dt) * (v_Cf_u[k] - v_Cf_u[k - 1]);

                // ═══════════════════════════════════════════════════════
                // V-phase solve (3×3 block, design doc eq. (7) + sequential)
                // ═══════════════════════════════════════════════════════
                double B1_v = i_Lf_v[k - 1] + a * VoltageV[k];
                double B2_v = v_Cf_v[k - 1];
                double B3_v = i_m_v[k - 1] - c * ev;
                double B2pbB1_v = B2_v + b * B1_v;

                // Step 1: motor current
                i_m_v[k] = (B3_v * K_ab + c * B2pbB1_v) * Dinv;

                // Step 2: capacitor voltage from motor equation
                v_Cf_v[k] = (Leq / dt) * (alpha * i_m_v[k] - i_m_v[k - 1]) + ev;

                // Step 3: inductor current from capacitor equation
                i_Lf_v[k] = i_m_v[k] + (Cf / dt) * (v_Cf_v[k] - v_Cf_v[k - 1]);
            }

            // ── W-phase from Y-connection constraints (design doc eq. 10a–10c) ──
            double[] i_m_w = new double[len];
            double[] v_Cf_w = new double[len];
            double[] i_Lf_w = new double[len];
            for (int k = 0; k < len; k++)
            {
                i_m_w[k]  = -i_m_u[k]  - i_m_v[k];
                v_Cf_w[k] = -v_Cf_u[k] - v_Cf_v[k];
                i_Lf_w[k] = -i_Lf_u[k] - i_Lf_v[k];
            }

            // ── Store results ──
            CurrentU = new List<double>(i_m_u);
            CurrentV = new List<double>(i_m_v);
            CurrentW = new List<double>(i_m_w);
            MotorVoltageU = new List<double>(v_Cf_u);
            MotorVoltageV = new List<double>(v_Cf_v);
            MotorVoltageW = new List<double>(v_Cf_w);
            FilterInductorCurrentU = new List<double>(i_Lf_u);
            FilterInductorCurrentV = new List<double>(i_Lf_v);
            FilterInductorCurrentW = new List<double>(i_Lf_w);

            // ── FFT of last ~10 periods for spectral display ──
            double period = 1.0 / MotorData.BaseFrequency;
            List<double> _iu = new List<double> { };
            List<double> _uu = new List<double> { };
            List<double> _uuv = new List<double> { };
            double _t_count = 0.0;
            for (int i = CurrentU.Count - 1; i > 0; i--)
            {
                if (_t_count >= 10.0 * period) break;
                _iu.Add(CurrentU[i]);
                _uu.Add(VoltageU[i]);
                _uuv.Add(VoltageUV[i]);
                _t_count += dt;
            }

            double[] _iu_array = _iu.ToArray();
            double[] _uu_array = _uu.ToArray();
            double[] _uuv_array = _uuv.ToArray();
            Array.Reverse(_iu_array);
            Array.Reverse(_uu_array);
            Array.Reverse(_uuv_array);

            FFTContainer _fft_iu = FFTOperations.GetFFT(_iu_array);
            FFTContainer _fft_uu = FFTOperations.GetFFT(_uu_array);
            FFTContainer _fft_uuv = FFTOperations.GetFFT(_uuv_array);

            List<double> _amp_iu = new List<double> { };
            List<double> _freq_iu = new List<double> { };
            for (int i = 0; i < _fft_iu.Amplitude.Count; i++)
            {
                int _order = _fft_iu.Order[i];
                double _actual_order = _order / 10.0;
                _amp_iu.Add(_fft_iu.Amplitude[i]);
                _freq_iu.Add(_actual_order * MotorData.BaseFrequency);
            }
            CurrentAmplitudes = _amp_iu;
            CurrentFrequencies = _freq_iu;

            List<double> _amp_uu = new List<double> { };
            List<double> _freq_uu = new List<double> { };
            for (int i = 0; i < _fft_uu.Amplitude.Count; i++)
            {
                int _order = _fft_uu.Order[i];
                double _actual_order = _order / 10.0;
                _amp_uu.Add(_fft_uu.Amplitude[i]);
                _freq_uu.Add(_actual_order * MotorData.BaseFrequency);
            }
            VoltageAmplitudes = _amp_uu;
            VoltageFrequencies = _freq_uu;

            List<double> _amp_uuv = new List<double> { };
            List<double> _freq_uuv = new List<double> { };
            for (int i = 0; i < _fft_uuv.Amplitude.Count; i++)
            {
                int _order = _fft_uuv.Order[i];
                double _actual_order = _order / 10.0;
                _amp_uuv.Add(_fft_uuv.Amplitude[i]);
                _freq_uuv.Add(_actual_order * MotorData.BaseFrequency);
            }
            LineToLineVoltageAmplitudes = _amp_uuv;
            LineToLineVoltageFrequencies = _freq_uuv;
        }
    }


    /// <summary>
    /// DQ-frame transient current solver for Y-connected PMSM (star winding).
    /// Uses 2×2 backward Euler discretization of the dq voltage equations.
    /// Ld and Lq are constant parameters — no per-step inductance updates needed.
    /// </summary>
    public class DQTransientSolverStar : Solver
    {
        public PMSMdq Motor { get; private set; }
        public double IdRMS { get; private set; }
        public double IqRMS { get; private set; }

        /// <summary>
        /// Initial d-axis current (A). Default is 0.0 (cold start).
        /// Setting this to the steady-state Id eliminates the DC transient offset.
        /// </summary>
        private double _id_init;
        /// <summary>
        /// Initial q-axis current (A). Default is 0.0 (cold start).
        /// Setting this to the steady-state Iq eliminates the DC transient offset.
        /// </summary>
        private double _iq_init;

        public DQTransientSolverStar(PMSMdq motor,
            List<double> voltageU, List<double> voltageV, List<double> voltageW,
            List<double> time, int periods,
            double id_init = 0.0, double iq_init = 0.0)
            : base(voltageU, voltageV, voltageW, time, periods)
        {
            this.Motor = motor;
            this._id_init = id_init;
            this._iq_init = iq_init;
            _getCurrents();
        }

        private void _getCurrents()
        {
            double R = Motor.PhaseResistance;
            double Ld = Motor.DInductance;
            double Lq = Motor.QInductance;
            double psi = Motor.PMFluxLinkage;
            double omega = Motor.AngularSpeed;

            int len = this.Time.Count;
            if (len < 2)
                throw new InvalidOperationException("Time vector must contain at least 2 points.");
            double dt = Time[1] - Time[0];

            // ── Precompute 2×2 coefficients (all loop-invariant) ──
            double a11 = 1.0 + dt * R / Ld;
            double a12 = -dt * omega * Lq / Ld;
            double a21 = dt * omega * Ld / Lq;
            double a22 = 1.0 + dt * R / Lq;
            double det = a11 * a22 - a12 * a21;
            double det_inv = 1.0 / det;

            // ── Park transform constants ──
            double sqrt3 = Math.Sqrt(3.0);
            double half_sqrt3 = sqrt3 / 2.0;

            // ── Angle recurrence for Park transform ──
            double sin_a = 0.0;
            double cos_a = 1.0;
            double cos_dt = Math.Cos(omega * dt);
            double sin_dt = Math.Sin(omega * dt);

            // ── Preallocate ──
            CurrentU = new List<double>(len) { 0.0 };
            CurrentV = new List<double>(len) { 0.0 };
            CurrentW = new List<double>(len) { 0.0 };

            // ── State ──
            double id_prev = _id_init, iq_prev = _iq_init;
            double sum_id_sq = 0.0, sum_iq_sq = 0.0;

            // Store DQ currents per step for synchronous demodulation
            double[] id_all = new double[len];
            double[] iq_all = new double[len];
            id_all[0] = _id_init;
            iq_all[0] = _iq_init;

            for (int k = 1; k < len; k++)
            {
                // Advance angle θ via rotation recurrence
                double sn = sin_a * cos_dt + cos_a * sin_dt;
                double cs = cos_a * cos_dt - sin_a * sin_dt;
                sin_a = sn;
                cos_a = cs;

                // Park: ABC → DQ (zero-sequence-rejecting Clarke — cancels 3rd harmonic common-mode)
                double v_alpha = (2.0 * VoltageU[k] - VoltageV[k] - VoltageW[k]) / 3.0;
                double v_beta = (VoltageV[k] - VoltageW[k]) / sqrt3;
                double vd =  v_alpha * cs + v_beta * sn;
                double vq = -v_alpha * sn + v_beta * cs;

                // RHS
                double b1 = id_prev + dt * vd / Ld;
                double b2 = iq_prev + dt * (vq - omega * psi) / Lq;

                // Solve 2×2 (Cramer's rule)
                double id = (a22 * b1 - a12 * b2) * det_inv;
                double iq = (a11 * b2 - a21 * b1) * det_inv;

                // Inverse Park: DQ → ABC
                double i_alpha = id * cs - iq * sn;
                double i_beta  = id * sn + iq * cs;
                double iu = i_alpha;
                double iv = -0.5 * i_alpha + half_sqrt3 * i_beta;
                double iw = -0.5 * i_alpha - half_sqrt3 * i_beta;

                CurrentU.Add(iu);
                CurrentV.Add(iv);
                CurrentW.Add(iw);

                id_all[k] = id;
                iq_all[k] = iq;
                id_prev = id;
                iq_prev = iq;
                sum_id_sq += id * id;
                sum_iq_sq += iq * iq;
            }

            // RMS values
            int n_active = len - 1;
            IdRMS = Math.Sqrt(sum_id_sq / n_active);
            IqRMS = Math.Sqrt(sum_iq_sq / n_active);

            // ── Synchronous demodulation: DC-mean of DQ currents over steady-state window ──
            // This extracts the true fundamental without spectral leakage from switching
            // harmonics, because the DQ frame rotates synchronously with the fundamental.
            // Switching harmonics appear as AC ripple that averages to zero.
            int ssPeriods = periods < 12 ? 1 : 4;
            double period = 1.0 / Motor.BaseFrequency;
            double ssDuration = ssPeriods * period;
            int ssStart = len - 1;
            double tSum = 0.0;
            // Walk backwards to find the start of the steady-state window
            for (int k = len - 1; k >= 1; k--)
            {
                tSum += dt;
                ssStart = k;
                if (tSum >= ssDuration) break;
            }
            int ssCount = len - ssStart;
            if (ssCount < 1) ssCount = 1;
            double sum_id = 0.0, sum_iq = 0.0;
            for (int k = ssStart; k < len; k++)
            {
                sum_id += id_all[k];
                sum_iq += iq_all[k];
            }
            IdFund = sum_id / ssCount;
            IqFund = sum_iq / ssCount;

            // ── FFT of last ~10 periods ──
            List<double> _iu = new List<double> { };
            List<double> _uu = new List<double> { };
            List<double> _uuv = new List<double> { };
            double _t_count = 0.0;
            for (int i = CurrentU.Count - 1; i > 0; i--)
            {
                if (_t_count >= 10.0 * period) break;
                _iu.Add(CurrentU[i]);
                _uu.Add(VoltageU[i]);
                _uuv.Add(VoltageUV[i]);
                _t_count += dt;
            }

            double[] _iu_array = _iu.ToArray();
            double[] _uu_array = _uu.ToArray();
            double[] _uuv_array = _uuv.ToArray();
            Array.Reverse(_iu_array);
            Array.Reverse(_uu_array);
            Array.Reverse(_uuv_array);

            FFTContainer _fft_iu = FFTOperations.GetFFT(_iu_array);
            FFTContainer _fft_uu = FFTOperations.GetFFT(_uu_array);
            FFTContainer _fft_uuv = FFTOperations.GetFFT(_uuv_array);

            List<double> _amp_iu = new List<double> { };
            List<double> _freq_iu = new List<double> { };
            for (int i = 0; i < _fft_iu.Amplitude.Count; i++)
            {
                int _order = _fft_iu.Order[i];
                double _actual_order = _order / 10.0;
                _amp_iu.Add(_fft_iu.Amplitude[i]);
                _freq_iu.Add(_actual_order * Motor.BaseFrequency);
            }
            CurrentAmplitudes = _amp_iu;
            CurrentFrequencies = _freq_iu;

            List<double> _amp_uu = new List<double> { };
            List<double> _freq_uu = new List<double> { };
            for (int i = 0; i < _fft_uu.Amplitude.Count; i++)
            {
                int _order = _fft_uu.Order[i];
                double _actual_order = _order / 10.0;
                _amp_uu.Add(_fft_uu.Amplitude[i]);
                _freq_uu.Add(_actual_order * Motor.BaseFrequency);
            }
            VoltageAmplitudes = _amp_uu;
            VoltageFrequencies = _freq_uu;

            List<double> _amp_uuv = new List<double> { };
            List<double> _freq_uuv = new List<double> { };
            for (int i = 0; i < _fft_uuv.Amplitude.Count; i++)
            {
                int _order = _fft_uuv.Order[i];
                double _actual_order = _order / 10.0;
                _amp_uuv.Add(_fft_uuv.Amplitude[i]);
                _freq_uuv.Add(_actual_order * Motor.BaseFrequency);
            }
            LineToLineVoltageAmplitudes = _amp_uuv;
            LineToLineVoltageFrequencies = _freq_uuv;
        }
    }


    /// <summary>
    /// Full transient solver with LC output filter in DQ frame.
    /// 6-state coupled system solved via sequential semi-implicit approach:
    /// motor 2×2 → capacitor → inductor → corrector.
    /// </summary>
    public class DQTransientSolverWithLCFilterFull : Solver
    {
        public List<double> MotorVoltageU { get; private set; }
        public List<double> MotorVoltageV { get; private set; }
        public List<double> MotorVoltageW { get; private set; }

        public List<double> FilterInductorCurrentU { get; private set; }
        public List<double> FilterInductorCurrentV { get; private set; }
        public List<double> FilterInductorCurrentW { get; private set; }

        /// <summary>
        /// Create a full transient DQ LC+motor coupled solver.
        /// </summary>
        /// <param name="motor">dq-frame motor model</param>
        /// <param name="pwmU">Raw PWM phase-U voltage (V)</param>
        /// <param name="pwmV">Raw PWM phase-V voltage (V)</param>
        /// <param name="pwmW">Raw PWM phase-W voltage (V)</param>
        /// <param name="time">Time vector (s)</param>
        /// <param name="periods">Number of fundamental periods</param>
        /// <param name="Lf">Filter inductance per phase (H)</param>
        /// <param name="Cf">Filter capacitance per phase (F), Y-connected</param>
        public DQTransientSolverWithLCFilterFull(PMSMdq motor,
            List<double> pwmU, List<double> pwmV, List<double> pwmW,
            List<double> time, int periods,
            double Lf, double Cf)
            : base(pwmU, pwmV, pwmW, time, periods)
        {
            if (Lf <= 0)
                throw new ArgumentException("Filter inductance must be positive.", nameof(Lf));
            if (Cf <= 0)
                throw new ArgumentException("Filter capacitance must be positive.", nameof(Cf));
            _getCurrents(motor, Lf, Cf);
        }

        private void _getCurrents(PMSMdq motor, double Lf, double Cf)
        {
            double R = motor.PhaseResistance;
            double Ld = motor.DInductance;
            double Lq = motor.QInductance;
            double psi = motor.PMFluxLinkage;
            double omega = motor.AngularSpeed;

            int len = Time.Count;
            if (len < 2)
                throw new InvalidOperationException("Time vector must contain at least 2 points.");
            double dt = Time[1] - Time[0];

            // ── Precompute time-invariant constants ──
            double a_f = dt / Lf;       // inductor coefficient
            double b_f = dt / Cf;       // capacitor coefficient
            double c_d = dt / Ld;       // motor d-axis coefficient
            double c_q = dt / Lq;       // motor q-axis coefficient

            // Motor 2×2 coefficients (solved with v_Cf as input)
            double a11 = 1.0 + c_d * R;
            double a12 = -c_d * omega * Lq;
            double a21 = c_q * omega * Ld;
            double a22 = 1.0 + c_q * R;
            double det_m = a11 * a22 - a12 * a21;
            double det_inv = 1.0 / det_m;

            // ── Park transform constants ──
            double sqrt3 = Math.Sqrt(3.0);
            double half_sqrt3 = sqrt3 / 2.0;

            // ── Angle recurrence ──
            double sin_a = 0.0;
            double cos_a = 1.0;
            double cos_dt = Math.Cos(omega * dt);
            double sin_dt = Math.Sin(omega * dt);

            // ── State arrays ──
            double[] i_Lf_d = new double[len];
            double[] i_Lf_q = new double[len];
            double[] v_Cf_d = new double[len];
            double[] v_Cf_q = new double[len];
            double[] i_m_d = new double[len];
            double[] i_m_q = new double[len];

            // All initial values are 0.0 (default)

            for (int k = 1; k < len; k++)
            {
                // Advance angle
                double sn = sin_a * cos_dt + cos_a * sin_dt;
                double cs = cos_a * cos_dt - sin_a * sin_dt;
                sin_a = sn;
                cos_a = cs;

                // ── Park: ABC → DQ (inverter voltage, zero-sequence-rejecting Clarke) ──
                double v_alpha = (2.0 * VoltageU[k] - VoltageV[k] - VoltageW[k]) / 3.0;
                double v_beta = (VoltageV[k] - VoltageW[k]) / sqrt3;
                double v_inv_d =  v_alpha * cs + v_beta * sn;
                double v_inv_q = -v_alpha * sn + v_beta * cs;

                // ══════════════════════════════════════════════════
                // Step A: Motor 2×2 solve with v_Cf from previous step (predictor)
                // ══════════════════════════════════════════════════
                double b1 = i_m_d[k - 1] + c_d * v_Cf_d[k - 1];
                double b2 = i_m_q[k - 1] + c_q * (v_Cf_q[k - 1] - omega * psi);

                i_m_d[k] = (a22 * b1 - a12 * b2) * det_inv;
                i_m_q[k] = (a11 * b2 - a21 * b1) * det_inv;

                // ══════════════════════════════════════════════════
                // Step B: Capacitor update (explicit Euler for cross terms)
                // ══════════════════════════════════════════════════
                v_Cf_d[k] = v_Cf_d[k - 1] + b_f * (i_Lf_d[k - 1] - i_m_d[k]) + dt * omega * v_Cf_q[k - 1];
                v_Cf_q[k] = v_Cf_q[k - 1] + b_f * (i_Lf_q[k - 1] - i_m_q[k]) - dt * omega * v_Cf_d[k - 1];

                // ══════════════════════════════════════════════════
                // Step C: Inductor update (explicit Euler for cross terms)
                // ══════════════════════════════════════════════════
                i_Lf_d[k] = i_Lf_d[k - 1] + a_f * (v_inv_d - v_Cf_d[k]) + dt * omega * i_Lf_q[k - 1];
                i_Lf_q[k] = i_Lf_q[k - 1] + a_f * (v_inv_q - v_Cf_q[k]) - dt * omega * i_Lf_d[k - 1];

                // ══════════════════════════════════════════════════
                // Step D: Corrector — re-solve motor with updated v_Cf
                // ══════════════════════════════════════════════════
                double b1c = i_m_d[k - 1] + c_d * v_Cf_d[k];
                double b2c = i_m_q[k - 1] + c_q * (v_Cf_q[k] - omega * psi);
                i_m_d[k] = (a22 * b1c - a12 * b2c) * det_inv;
                i_m_q[k] = (a11 * b2c - a21 * b1c) * det_inv;
            }

            // ── Convert DQ → ABC for all states ──
            // We need to recompute sin/cos for each step (or store them).
            // For simplicity, we re-run the angle recurrence and do inverse Park.
            sin_a = 0.0;
            cos_a = 1.0;

            double[] i_m_u = new double[len];
            double[] i_m_v = new double[len];
            double[] i_m_w = new double[len];
            double[] v_Cf_u = new double[len];
            double[] v_Cf_v = new double[len];
            double[] v_Cf_w = new double[len];
            double[] i_Lf_u = new double[len];
            double[] i_Lf_v = new double[len];
            double[] i_Lf_w = new double[len];

            for (int k = 0; k < len; k++)
            {
                if (k > 0)
                {
                    double sn = sin_a * cos_dt + cos_a * sin_dt;
                    double cs = cos_a * cos_dt - sin_a * sin_dt;
                    sin_a = sn;
                    cos_a = cs;
                }

                // Inverse Park: DQ → αβ → ABC
                double i_alpha = i_m_d[k] * cos_a - i_m_q[k] * sin_a;
                double i_beta  = i_m_d[k] * sin_a + i_m_q[k] * cos_a;
                i_m_u[k] = i_alpha;
                i_m_v[k] = -0.5 * i_alpha + half_sqrt3 * i_beta;
                i_m_w[k] = -0.5 * i_alpha - half_sqrt3 * i_beta;

                double v_alpha = v_Cf_d[k] * cos_a - v_Cf_q[k] * sin_a;
                double v_beta  = v_Cf_d[k] * sin_a + v_Cf_q[k] * cos_a;
                v_Cf_u[k] = v_alpha;
                v_Cf_v[k] = -0.5 * v_alpha + half_sqrt3 * v_beta;
                v_Cf_w[k] = -0.5 * v_alpha - half_sqrt3 * v_beta;

                double l_alpha = i_Lf_d[k] * cos_a - i_Lf_q[k] * sin_a;
                double l_beta  = i_Lf_d[k] * sin_a + i_Lf_q[k] * cos_a;
                i_Lf_u[k] = l_alpha;
                i_Lf_v[k] = -0.5 * l_alpha + half_sqrt3 * l_beta;
                i_Lf_w[k] = -0.5 * l_alpha - half_sqrt3 * l_beta;
            }

            // ── Store results ──
            CurrentU = new List<double>(i_m_u);
            CurrentV = new List<double>(i_m_v);
            CurrentW = new List<double>(i_m_w);
            MotorVoltageU = new List<double>(v_Cf_u);
            MotorVoltageV = new List<double>(v_Cf_v);
            MotorVoltageW = new List<double>(v_Cf_w);
            FilterInductorCurrentU = new List<double>(i_Lf_u);
            FilterInductorCurrentV = new List<double>(i_Lf_v);
            FilterInductorCurrentW = new List<double>(i_Lf_w);

            // ── Synchronous demodulation: DC-mean of DQ currents over steady-state window ──
            int ssPeriodsFull = periods < 12 ? 1 : 4;
            double ssPeriod = 1.0 / motor.BaseFrequency;
            double ssDurationFull = ssPeriodsFull * ssPeriod;
            int ssStartFull = len - 1;
            double tSumFull = 0.0;
            for (int k = len - 1; k >= 1; k--)
            {
                tSumFull += dt;
                ssStartFull = k;
                if (tSumFull >= ssDurationFull) break;
            }
            int ssCountFull = len - ssStartFull;
            if (ssCountFull < 1) ssCountFull = 1;
            double sum_id_full = 0.0, sum_iq_full = 0.0;
            for (int k = ssStartFull; k < len; k++)
            {
                sum_id_full += i_m_d[k];
                sum_iq_full += i_m_q[k];
            }
            IdFund = sum_id_full / ssCountFull;
            IqFund = sum_iq_full / ssCountFull;

            // ── FFT of last ~10 periods ──
            double period = 1.0 / motor.BaseFrequency;
            List<double> _iu = new List<double> { };
            List<double> _uu = new List<double> { };
            List<double> _uuv = new List<double> { };
            double _t_count = 0.0;
            for (int i = CurrentU.Count - 1; i > 0; i--)
            {
                if (_t_count >= 10.0 * period) break;
                _iu.Add(CurrentU[i]);
                _uu.Add(MotorVoltageU[i]);
                _uuv.Add(MotorVoltageU[i] - MotorVoltageV[i]);
                _t_count += dt;
            }

            double[] _iu_array = _iu.ToArray();
            double[] _uu_array = _uu.ToArray();
            double[] _uuv_array = _uuv.ToArray();
            Array.Reverse(_iu_array);
            Array.Reverse(_uu_array);
            Array.Reverse(_uuv_array);

            FFTContainer _fft_iu = FFTOperations.GetFFT(_iu_array);
            FFTContainer _fft_uu = FFTOperations.GetFFT(_uu_array);
            FFTContainer _fft_uuv = FFTOperations.GetFFT(_uuv_array);

            List<double> _amp_iu = new List<double> { };
            List<double> _freq_iu = new List<double> { };
            for (int i = 0; i < _fft_iu.Amplitude.Count; i++)
            {
                int _order = _fft_iu.Order[i];
                double _actual_order = _order / 10.0;
                _amp_iu.Add(_fft_iu.Amplitude[i]);
                _freq_iu.Add(_actual_order * motor.BaseFrequency);
            }
            CurrentAmplitudes = _amp_iu;
            CurrentFrequencies = _freq_iu;

            List<double> _amp_uu = new List<double> { };
            List<double> _freq_uu = new List<double> { };
            for (int i = 0; i < _fft_uu.Amplitude.Count; i++)
            {
                int _order = _fft_uu.Order[i];
                double _actual_order = _order / 10.0;
                _amp_uu.Add(_fft_uu.Amplitude[i]);
                _freq_uu.Add(_actual_order * motor.BaseFrequency);
            }
            VoltageAmplitudes = _amp_uu;
            VoltageFrequencies = _freq_uu;

            List<double> _amp_uuv = new List<double> { };
            List<double> _freq_uuv = new List<double> { };
            for (int i = 0; i < _fft_uuv.Amplitude.Count; i++)
            {
                int _order = _fft_uuv.Order[i];
                double _actual_order = _order / 10.0;
                _amp_uuv.Add(_fft_uuv.Amplitude[i]);
                _freq_uuv.Add(_actual_order * motor.BaseFrequency);
            }
            LineToLineVoltageAmplitudes = _amp_uuv;
            LineToLineVoltageFrequencies = _freq_uuv;
        }
    }


    public enum SolverType
    {
        /// <summary>DQ 2×2 backward Euler transient solver (no LC filter).</summary>
        Transient,
        /// <summary>[Obsolete] Former ABC harmonic AC solver. Now redirected to DQTransientSolverStar with steady-state ICs.</summary>
        AC,
        /// <summary>LEGACY: ABC 6-state coupled LC+motor transient solver.</summary>
        TransientWithLCFilter,
        /// <summary>DQ 6-state fully coupled LC+motor transient solver.</summary>
        TransientWithLCFilterFull
    }

    public class DriveStarConnected
    {
        private PMSMWithConstantInductance motorDataWithConstantInductance;
        private PMSMWithVariableInductance motorDataWithVariableInductance;

        private IPSPWM3 pwmGeneratorIPSPWM3;
        private SVPWM3 pwmGeneratorSVPWM3;
        private SPWM2 pwmGeneratorSPWM2;
        private SVPWM2 pwmGeneratorSVPWM2;
        private QuasiSVPWM3 pwmGeneratorQuasiSVPWM3;
        private QuasiSVPWM2 pwmGeneratorQuasiSVPWM2;

        private TransientSolverStar transientSolverStar;
        private ACSolverStar acSolverStar;

        private SolverType solverType;
        private double pwmFrequency;

        public Solver Result { get; private set; }

        public void SetPMSM(PMSMWithConstantInductance motorData)
        {
            this.motorDataWithConstantInductance = motorData;
            this.motorDataWithVariableInductance = null;
        }

        public void SetPMSM(PMSMWithVariableInductance motorData)
        {
            this.motorDataWithVariableInductance = motorData;
            this.motorDataWithConstantInductance = null;
        }

        public void SetPWM(IPSPWM3 pwmGenerator)
        {
            this.pwmGeneratorIPSPWM3 = pwmGenerator;
            this.pwmGeneratorSVPWM3 = null;
            this.pwmGeneratorSPWM2 = null;
            this.pwmGeneratorSVPWM2 = null;
            this.pwmGeneratorQuasiSVPWM2 = null;
            this.pwmGeneratorQuasiSVPWM3 = null;
        }

        public void SetPWM(SVPWM3 pwmGenerator)
        {
            this.pwmGeneratorSVPWM3 = pwmGenerator;
            this.pwmGeneratorIPSPWM3 = null;
            this.pwmGeneratorSPWM2 = null;
            this.pwmGeneratorSVPWM2 = null;
            this.pwmGeneratorQuasiSVPWM2 = null;
            this.pwmGeneratorQuasiSVPWM3 = null;
        }

        public void SetPWM(SPWM2 pwmGenerator)
        {
            this.pwmGeneratorSPWM2 = pwmGenerator;
            this.pwmGeneratorSVPWM3 = null;
            this.pwmGeneratorIPSPWM3 = null;
            this.pwmGeneratorSVPWM2 = null;
            this.pwmGeneratorQuasiSVPWM2 = null;
            this.pwmGeneratorQuasiSVPWM3 = null;
        }

        public void SetPWM(SVPWM2 pwmGenerator)
        {
            this.pwmGeneratorSVPWM2 = pwmGenerator;
            this.pwmGeneratorSPWM2 = null;
            this.pwmGeneratorSVPWM3 = null;
            this.pwmGeneratorIPSPWM3 = null;
            this.pwmGeneratorQuasiSVPWM2 = null;
            this.pwmGeneratorQuasiSVPWM3 = null;
        }


        public void SetPWM(QuasiSVPWM3 pwmGenerator)
        {
            this.pwmGeneratorSVPWM2 = null;
            this.pwmGeneratorSPWM2 = null;
            this.pwmGeneratorSVPWM3 = null;
            this.pwmGeneratorIPSPWM3 = null;
            this.pwmGeneratorQuasiSVPWM2 = null;
            this.pwmGeneratorQuasiSVPWM3 = pwmGenerator;
        }

        public void SetPWM(QuasiSVPWM2 pwmGenerator)
        {
            this.pwmGeneratorSVPWM2 = null;
            this.pwmGeneratorSPWM2 = null;
            this.pwmGeneratorSVPWM3 = null;
            this.pwmGeneratorIPSPWM3 = null;
            this.pwmGeneratorQuasiSVPWM2 = pwmGenerator;
            this.pwmGeneratorQuasiSVPWM3 = null;
        }


        public void SetSolver(SolverType type)
        {
            this.solverType = type;
        }

        public void Solve(double peakPhaseVoltage, double voltageAngleDegInDQ,
            double speed, int periods)
        {
            if (this.motorDataWithConstantInductance != null)
            {
                this.motorDataWithConstantInductance.SpeedRPM = speed;
            }
            else this.motorDataWithVariableInductance.SpeedRPM = speed;

            double fundamentalFrequency = (this.motorDataWithConstantInductance == null) ?
                this.motorDataWithVariableInductance.BaseFrequency :
                this.motorDataWithConstantInductance.BaseFrequency;

            double _theta0 = (voltageAngleDegInDQ - 90.0) / 180.0 * Math.PI;
            List<double> uu, uv, uw, time;
            if (this.pwmGeneratorIPSPWM3 != null)
            {
                pwmFrequency = this.pwmGeneratorIPSPWM3.SwitchingFrequency;
                List<List<double>> _voltageU = this.pwmGeneratorIPSPWM3.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0,
                    periods);
                List<List<double>> _voltageV = this.pwmGeneratorIPSPWM3.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0 - 2.0 / 3.0 * Math.PI,
                    periods);
                List<List<double>> _voltageW = this.pwmGeneratorIPSPWM3.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0 - 4.0 / 3.0 * Math.PI,
                    periods);

                time = _voltageU[0];
                uu = _voltageU[1];
                uv = _voltageV[1];
                uw = _voltageW[1];
            }
            else if (this.pwmGeneratorSPWM2 != null)
            {
                pwmFrequency = this.pwmGeneratorSPWM2.SwitchingFrequency;
                List<List<double>> _voltageU = this.pwmGeneratorSPWM2.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0,
                    periods);
                List<List<double>> _voltageV = this.pwmGeneratorSPWM2.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0 - 2.0 / 3.0 * Math.PI,
                    periods);
                List<List<double>> _voltageW = this.pwmGeneratorSPWM2.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0 - 4.0 / 3.0 * Math.PI,
                    periods);

                time = _voltageU[0];
                uu = _voltageU[1];
                uv = _voltageV[1];
                uw = _voltageW[1];
            }
            else if (this.pwmGeneratorSVPWM3 != null)
            {
                pwmFrequency = this.pwmGeneratorSVPWM3.SwitchingFrequency;
                List<List<double>> _uUVWToBeCorrected = this.pwmGeneratorSVPWM3.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0, periods);
                FFTContainer _fftCorrection = FFTOperations.GetFFT(_uUVWToBeCorrected[1].ToArray());
                double _offset = voltageAngleDegInDQ - 180 - _fftCorrection.Phase[periods];

                List<List<double>> _uUVW = this.pwmGeneratorSVPWM3.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0 + _offset / 180.0 * Math.PI,
                    periods);

                time = _uUVW[0];
                uu = _uUVW[1];
                uv = _uUVW[2];
                uw = _uUVW[3];
            }
            else if (this.pwmGeneratorSVPWM2 != null)
            {
                pwmFrequency = this.pwmGeneratorSVPWM2.SwitchingFrequency;
                List<List<double>> _uUVWToBeCorrected = this.pwmGeneratorSVPWM2.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0, periods);
                FFTContainer _fftCorrection = FFTOperations.GetFFT(_uUVWToBeCorrected[1].ToArray());
                double _offset = voltageAngleDegInDQ - 180 - _fftCorrection.Phase[periods];

                List<List<double>> _uUVW = this.pwmGeneratorSVPWM2.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0 + _offset / 180.0 * Math.PI,
                    periods);

                time = _uUVW[0];
                uu = _uUVW[1];
                uv = _uUVW[2];
                uw = _uUVW[3];
            }
            else if (this.pwmGeneratorQuasiSVPWM3 != null)
            {
                pwmFrequency = this.pwmGeneratorQuasiSVPWM3.SwitchingFrequency;
                List<List<double>> _voltageU = this.pwmGeneratorQuasiSVPWM3.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0,
                    periods);
                List<List<double>> _voltageV = this.pwmGeneratorQuasiSVPWM3.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0 - 2.0 / 3.0 * Math.PI,
                    periods);
                List<List<double>> _voltageW = this.pwmGeneratorQuasiSVPWM3.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0 - 4.0 / 3.0 * Math.PI,
                    periods);

                time = _voltageU[0];
                uu = _voltageU[1];
                uv = _voltageV[1];
                uw = _voltageW[1];
            }
            else
            {
                pwmFrequency = this.pwmGeneratorQuasiSVPWM2.SwitchingFrequency;
                List<List<double>> _voltageU = this.pwmGeneratorQuasiSVPWM2.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0,
                    periods);
                List<List<double>> _voltageV = this.pwmGeneratorQuasiSVPWM2.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0 - 2.0 / 3.0 * Math.PI,
                    periods);
                List<List<double>> _voltageW = this.pwmGeneratorQuasiSVPWM2.GetOutputVoltage(
                    peakPhaseVoltage, fundamentalFrequency, _theta0 - 4.0 / 3.0 * Math.PI,
                    periods);

                time = _voltageU[0];
                uu = _voltageU[1];
                uv = _voltageV[1];
                uw = _voltageW[1];
            }

            switch (this.solverType)
            {
                case SolverType.Transient:
                    if (this.motorDataWithConstantInductance != null)
                    {
                        this.transientSolverStar = new TransientSolverStar(
                            this.motorDataWithConstantInductance, uu, uv, uw, time, periods);
                    }
                    else
                    {
                        this.transientSolverStar = new TransientSolverStar(
                            this.motorDataWithVariableInductance, uu, uv, uw, time, periods);
                    }
                    this.acSolverStar = null;
                    this.Result = (Solver)this.transientSolverStar;
                    break;

                case SolverType.AC:
                    if (this.motorDataWithConstantInductance != null)
                    {
                        this.acSolverStar = new ACSolverStar(this.motorDataWithConstantInductance,
                            uu, uv, uw, time, periods, pwmFrequency);
                    }
                    else
                    {
                        this.acSolverStar = new ACSolverStar(this.motorDataWithVariableInductance,
                            uu, uv, uw, time, periods, pwmFrequency);
                    }
                    this.transientSolverStar = null;
                    this.Result = (Solver)this.acSolverStar;
                    break;
            }
        }
    }
}
