using System;
using System.Collections.Generic;
using System.Numerics;

namespace PMSMDriveCalc
{
    /// <summary>
    /// Steady-state dq-frame solver for PMSM.
    /// Given Ud, Uq and motor parameters, computes Id, Iq, torque, and phasor diagram data.
    /// This is purely analytical — no PWM or time-domain simulation involved.
    /// </summary>
    public class DQSteadyStateSolver
    {
        private PMSMdq motor;

        /// <summary>
        /// Create a steady-state dq solver for the given motor model.
        /// You must set SpeedRPM on the motor before calling solve methods.
        /// </summary>
        public DQSteadyStateSolver(PMSMdq motor)
        {
            this.motor = motor;
        }

        /// <summary>The PMSMdq motor model used for computation.</summary>
        public PMSMdq Motor { get => motor; set => motor = value; }

        /// <summary>
        /// Get the operating point currents (Id, Iq) for given dq voltages.
        /// </summary>
        public (double Id, double Iq) GetCurrents(double Ud, double Uq)
        {
            return motor.SolveSteadyStateCurrents(Ud, Uq);
        }

        /// <summary>
        /// Get the electromagnetic torque for given dq voltages.
        /// </summary>
        public double GetTorque(double Ud, double Uq)
        {
            return motor.CalculateSteadyStateTorque(Ud, Uq);
        }

        /// <summary>
        /// Get the complete operating point summary.
        /// </summary>
        public OperatingPointResult ComputeOperatingPoint(double Ud, double Uq)
        {
            var (Id, Iq) = motor.SolveSteadyStateCurrents(Ud, Uq);
            double torque = motor.CalculateTorque(Id, Iq);
            double vMag = motor.GetPhaseVoltageMagnitude(Ud, Uq);
            double vAngle = motor.GetVoltageAngleDQ(Ud, Uq);
            double iMag = Math.Sqrt(Id * Id + Iq * Iq);
            double iAngle = Math.Atan2(Iq, Id);
            double apparentPower = 1.5 * vMag * iMag;
            double activePower = 1.5 * (Ud * Id + Uq * Iq);
            double powerFactor = Math.Abs(apparentPower) > 1e-12 ?
                activePower / apparentPower : 1.0;
            double mechanicalPowerW = torque * motor.AngularSpeed / (motor.Poles / 2.0);
            double copperLossW = 1.5 * motor.PhaseResistance * (Id * Id + Iq * Iq);

            return new OperatingPointResult
            {
                Ud = Ud,
                Uq = Uq,
                Id = Id,
                Iq = Iq,
                TorqueNm = torque,
                PhaseVoltageMagnitude = vMag,
                VoltageAngleRad = vAngle,
                CurrentMagnitude = iMag,
                CurrentAngleRad = iAngle,
                ApparentPowerVA = apparentPower,
                PwmApparentPowerVA = apparentPower,
                ActivePowerW = activePower,
                PowerFactor = powerFactor,
                MechanicalPowerW = mechanicalPowerW,
                CopperLossW = copperLossW,
                MotorUd = Ud,
                MotorUq = Uq,
                ElectricalSpeedRadS = motor.AngularSpeed,
                ElectricalFreqHz = motor.BaseFrequency,
                SpeedRPM = motor.SpeedRPM,
                Poles = motor.Poles,
                PhaseResistance = motor.PhaseResistance,
                Ld = motor.DInductance,
                Lq = motor.QInductance,
                PMFluxLinkage = motor.PMFluxLinkage,
            };
        }

        /// <summary>
        /// Generate phasor diagram data for visualization.
        /// All phasors referenced to the d-axis (real axis).
        /// </summary>
        public PhasorDiagramData GetPhasorDiagram(double Ud, double Uq)
        {
            return motor.GetPhasorDiagram(Ud, Uq);
        }
    }

    /// <summary>
    /// Container for all operating-point results from a single dq calculation.
    /// </summary>
    public struct OperatingPointResult
    {
        // Inputs
        public double Ud;
        public double Uq;

        // Outputs — currents
        public double Id;
        public double Iq;

        // Torque
        public double TorqueNm;

        // Derived magnitudes and angles
        public double PhaseVoltageMagnitude;   // sqrt(Ud² + Uq²) peak phase voltage
        public double VoltageAngleRad;          // atan2(Uq, Ud) relative to d-axis
        public double CurrentMagnitude;         // sqrt(Id² + Iq²)
        public double CurrentAngleRad;          // atan2(Iq, Id) relative to d-axis

        // Power
        public double ApparentPowerVA;      // motor-side apparent power (after LC filter when active)
        public double PwmApparentPowerVA;   // inverter-side (PWM output) apparent power
        public double ActivePowerW;
        public double PowerFactor;

        // Mechanical
        public double MechanicalPowerW;     // P_mech = Torque × ω_mech = Torque × ElectricalSpeedRadS / (Poles/2)
        public double CopperLossW;          // 1.5 × R × (Id² + Iq²)

        // Motor terminal voltage info (filled when LC filter is active, same as Ud/Uq otherwise)
        public double MotorUd;              // actual d-axis motor terminal voltage (after LC filter)
        public double MotorUq;              // actual q-axis motor terminal voltage (after LC filter)

        // Speed / frequency info
        public double ElectricalSpeedRadS;
        public double ElectricalFreqHz;
        public double SpeedRPM;
        public int Poles;

        // Motor parameters (for reference)
        public double PhaseResistance;
        public double Ld;
        public double Lq;
        public double PMFluxLinkage;
    }

    /// <summary>
    /// Container for time-domain + frequency-domain simulation results
    /// from the full Ud/Uq → PWM → solver pipeline.
    /// </summary>
    public class PMSMDriveResult
    {
        /// <summary>Operating point from steady-state dq analysis.</summary>
        public OperatingPointResult OperatingPoint { get; set; }

        /// <summary>Time vector (seconds).</summary>
        public List<double> Time { get; set; }

        /// <summary>Phase U PWM voltage (V).</summary>
        public List<double> VU { get; set; }

        /// <summary>Phase V PWM voltage (V).</summary>
        public List<double> VV { get; set; }

        /// <summary>Phase W PWM voltage (V).</summary>
        public List<double> VW { get; set; }

        /// <summary>Phase U current (A).</summary>
        public List<double> IU { get; set; }

        /// <summary>Phase V current (A).</summary>
        public List<double> IV { get; set; }

        /// <summary>Phase W current (A).</summary>
        public List<double> IW { get; set; }

        /// <summary>FFT container for phase U voltage.</summary>
        public FFTContainer VU_FFT { get; set; }

        /// <summary>FFT container for phase V voltage.</summary>
        public FFTContainer VV_FFT { get; set; }

        /// <summary>FFT container for phase W voltage.</summary>
        public FFTContainer VW_FFT { get; set; }

        /// <summary>FFT container for phase U current.</summary>
        public FFTContainer IU_FFT { get; set; }

        /// <summary>FFT container for phase V current.</summary>
        public FFTContainer IV_FFT { get; set; }

        /// <summary>FFT container for phase W current.</summary>
        public FFTContainer IW_FFT { get; set; }

        /// <summary>Line-to-line voltage U-V (V).</summary>
        public List<double> VUV { get; set; }

        /// <summary>Line-to-line voltage V-W (V).</summary>
        public List<double> VVW { get; set; }

        /// <summary>Line-to-line voltage W-U (V).</summary>
        public List<double> VWU { get; set; }

        /// <summary>FFT container for line-to-line voltage U-V.</summary>
        public FFTContainer VUV_FFT { get; set; }

        /// <summary>FFT container for line-to-line voltage V-W.</summary>
        public FFTContainer VVW_FFT { get; set; }

        /// <summary>FFT container for line-to-line voltage W-U.</summary>
        public FFTContainer VWU_FFT { get; set; }

        /// <summary>Motor-side phase U voltage after LC filter (V).</summary>
        public List<double> MotorVU { get; set; }

        /// <summary>Motor-side phase V voltage after LC filter (V).</summary>
        public List<double> MotorVV { get; set; }

        /// <summary>Motor-side phase W voltage after LC filter (V).</summary>
        public List<double> MotorVW { get; set; }

        /// <summary>Motor-side line-to-line voltage U-V (V).</summary>
        public List<double> MotorVUV { get; set; }

        /// <summary>Motor-side line-to-line voltage V-W (V).</summary>
        public List<double> MotorVVW { get; set; }

        /// <summary>Motor-side line-to-line voltage W-U (V).</summary>
        public List<double> MotorVWU { get; set; }

        /// <summary>FFT container for motor-side phase U voltage.</summary>
        public FFTContainer MotorVU_FFT { get; set; }

        /// <summary>FFT container for motor-side phase V voltage.</summary>
        public FFTContainer MotorVV_FFT { get; set; }

        /// <summary>FFT container for motor-side phase W voltage.</summary>
        public FFTContainer MotorVW_FFT { get; set; }

        /// <summary>FFT container for motor-side line-to-line voltage U-V.</summary>
        public FFTContainer MotorVUV_FFT { get; set; }

        /// <summary>FFT container for motor-side line-to-line voltage V-W.</summary>
        public FFTContainer MotorVVW_FFT { get; set; }

        /// <summary>FFT container for motor-side line-to-line voltage W-U.</summary>
        public FFTContainer MotorVWU_FFT { get; set; }

        /// <summary>Base frequency used in the simulation (Hz).</summary>
        public double BaseFrequencyHz { get; set; }

        /// <summary>PWM line-to-line fundamental voltage, computed from dq command voltages (V).</summary>
        public double PwmVPhasePhaseFundamental { get; set; }

        /// <summary>Motor phase-to-neutral voltage RMS after LC filter (V). Null when LC filter is inactive.</summary>
        public double? MotorVPhaseNeutralRms { get; set; }

        /// <summary>Motor line-to-line fundamental voltage after LC filter, computed from motor dq voltages (V). Null when LC filter is inactive.</summary>
        public double? MotorVPhasePhaseFundamental { get; set; }
    }

    /// <summary>
    /// Full pipeline calculator: Ud/Uq → 3-phase references → PWM → solver → waveforms + FFT.
    /// This is the main entry point for external callers that want to drive the PMSM
    /// simulation purely from dq-voltage commands.
    ///
    /// Usage example:
    /// <code>
    /// var motor = new PMSMdq(4, 0.1, 0.005, 0.0055, 0.5);
    /// motor.SpeedRPM = 1500;
    /// var pwm = new SVPWM2(8000, 600);
    /// var calc = new PMSMDQDriveCalculator(motor, pwm);
    /// var result = calc.Compute(100, 150, SolverType.AC, 20);
    /// // Access: result.IU, result.IU_FFT, result.OperatingPoint, etc.
    /// </code>
    /// </summary>
    public class PMSMDQDriveCalculator
    {
        private PMSMdq dqMotor;
        private PMSMWithConstantInductance abcMotor;
        private ICanOutputVoltage pwm;
        private PWM pwmBase;

        /// <summary>
        /// Create a DQ-drive calculator.
        /// </summary>
        /// <param name="dqMotor">dq-frame motor model with Ld, Lq</param>
        /// <param name="pwm">PWM modulation strategy</param>
        public PMSMDQDriveCalculator(PMSMdq dqMotor, ICanOutputVoltage pwm)
        {
            this.dqMotor = dqMotor;
            this.pwm = pwm;

            // Build an abc-frame equivalent motor model for the solver.
            // For a PMSM in dq frame with Ld and Lq:
            //   L_self  = (2·Ld + Lq)/3   (approximation for non-salient/slightly salient)
            //   L_mutual = (Ld - Lq)/3
            //   L_eq = L_self - L_mutual = (Ld + Lq)/2
            // This is the standard Y-connected PMSM transformation.
            double Ld = dqMotor.DInductance;
            double Lq = dqMotor.QInductance;
            double L_self = (2.0 * Ld + Lq) / 3.0;
            double L_mutual = (Ld - Lq) / 3.0;

            this.abcMotor = new PMSMWithConstantInductance(
                dqMotor.Poles,
                dqMotor.PMFluxLinkage,
                L_self,
                L_mutual,
                dqMotor.PhaseResistance
            );

            // Extract PWM base for frequency info
            if (pwm is PWM p)
                this.pwmBase = p;
            else
                this.pwmBase = null;
        }

        /// <summary>
        /// Get the steady-state dq solver (no PWM/simulation).
        /// </summary>
        public DQSteadyStateSolver SteadyState => new DQSteadyStateSolver(dqMotor);

        /// <summary>
        /// Compute the full drive simulation: steady-state operating point,
        /// 3-phase PWM voltages, time-domain currents, and FFT spectra.
        ///
        /// This runs the full pipeline:
        /// 1. Steady-state dq solve for Id, Iq, torque
        /// 2. Inverse Park + Clarke to get 3-phase sinusoidal references
        /// 3. PWM modulation to generate actual switching voltages
        /// 4. AC (frequency-domain) or Transient (time-stepping) solver
        /// 5. FFT of resulting currents and voltages
        /// </summary>
        /// <param name="Ud">d-axis voltage command (V)</param>
        /// <param name="Uq">q-axis voltage command (V)</param>
        /// <param name="solverType">AC (harmonic) or Transient (time-stepping)</param>
        /// <param name="periods">Number of fundamental periods to simulate</param>
        /// <returns>PMSMDriveResult with all waveforms and FFT data</returns>
        public PMSMDriveResult Compute(double Ud, double Uq,
            SolverType solverType, int periods)
        {
            // Sync speed from dq motor to abc motor
            abcMotor.SpeedRPM = dqMotor.SpeedRPM;

            // ---- Step 1: Steady-state operating point ----
            var op = SteadyState.ComputeOperatingPoint(Ud, Uq);
            double amplitudePeak = op.PhaseVoltageMagnitude;
            double voltageAngleRad = op.VoltageAngleRad;
            double freqHz = op.ElectricalFreqHz;

            // ---- Step 2 & 3: Generate 3-phase reference voltages and run PWM ----
            // The dq voltage vector is (Ud, Uq).
            // Inverse Park (dq → αβ) at electrical angle θ = ω·t:
            //   Vα = Ud·cos(ωt) - Uq·sin(ωt) = |V|·cos(ωt + φ_v)
            //   Vβ = Ud·sin(ωt) + Uq·cos(ωt) = |V|·sin(ωt + φ_v)
            //   where φ_v = atan2(Uq, Ud)
            //
            // Inverse Clarke (αβ → abc):
            //   Va = Vα = |V|·cos(ωt + φ_v)
            //   Vb = -Vα/2 + √3/2·Vβ = |V|·cos(ωt + φ_v - 2π/3)
            //   Vc = -Vα/2 - √3/2·Vβ = |V|·cos(ωt + φ_v - 4π/3)
            //
            // The existing DriveStarConnected uses sin-based references:
            //   Va = Amp·sin(ωt + phase) = Amp·cos(ωt + phase - π/2)
            // So phase_offset = φ_v - π/2 = atan2(Uq, Ud) - π/2
            //
            // However, for simplicity and correctness, we directly generate
            // the 3-phase reference voltages from dq quantities and pass them
            // through the PWM engine.

            // Get PWM output for each phase.
            // For PWM implementations that return 3-phase output (SVPWM family),
            // we call once. For single-phase output (SPWM family), we call per phase.
            List<double> time;
            List<double> uu, uv, uw;

            // Generate 3-phase reference voltages at a high sampling rate,
            // then use the PWM to generate actual switching waveforms.
            // The PWM's GetOutputVoltage takes (amplitudeVoltage, frequencyVoltage,
            // phaseVoltage, periods) and returns the switching waveform.
            //
            // For the dq→abc conversion, we need to generate the correct reference
            // for each phase. Phase U (A-axis) reference = |V|·cos(ωt + φ_v)
            // In sin convention: Phase U = |V|·sin(ωt + φ_v + π/2)

            double phaseOffset = voltageAngleRad + Math.PI / 2.0;

            // Try to get 3-phase output directly (SVPWM family)
            var output3ph = pwm.GetOutputVoltage(amplitudePeak, freqHz, phaseOffset, periods);

            if (output3ph.Count >= 4)
            {
                // 3-phase output: time, uU, uV, uW
                time = output3ph[0];
                uu = output3ph[1];
                uv = output3ph[2];
                uw = output3ph[3];
            }
            else
            {
                // Single-phase output: generate each phase separately
                time = output3ph[0];
                uu = output3ph[1];

                var phaseV = pwm.GetOutputVoltage(amplitudePeak, freqHz,
                    phaseOffset - 2.0 * Math.PI / 3.0, periods);
                var phaseW = pwm.GetOutputVoltage(amplitudePeak, freqHz,
                    phaseOffset - 4.0 * Math.PI / 3.0, periods);

                uv = phaseV[1];
                uw = phaseW[1];
            }

            // ---- Step 3b: When LC output filter is active, separate raw PWM from motor voltages ----
            // The uu/uv/uw from Step 3 are already motor-side voltages (after LC filter).
            // Get raw PWM voltages by unwrapping the OutputLCFilter to retrieve the original PWM output.
            List<double>? pwmU = null, pwmV = null, pwmW = null;
            if (pwm is OutputLCFilter lcFilter)
            {
                var rawPwmOutput = lcFilter.InnerPWM.GetOutputVoltage(amplitudePeak, freqHz, phaseOffset, periods);
                if (rawPwmOutput.Count >= 4)
                {
                    pwmU = rawPwmOutput[1];
                    pwmV = rawPwmOutput[2];
                    pwmW = rawPwmOutput[3];
                }
                else
                {
                    pwmU = rawPwmOutput[1];
                    var pV = lcFilter.InnerPWM.GetOutputVoltage(amplitudePeak, freqHz,
                        phaseOffset - 2.0 * Math.PI / 3.0, periods);
                    var pW = lcFilter.InnerPWM.GetOutputVoltage(amplitudePeak, freqHz,
                        phaseOffset - 4.0 * Math.PI / 3.0, periods);
                    pwmV = pV[1];
                    pwmW = pW[1];
                }
            }

            // ---- Step 4: Run current solver ----
            Solver solver;
            bool useCoupledLcSolver = false;

            switch (solverType)
            {
                case SolverType.AC:
                {
                    // Use initialized DQ transient solver instead of the old
                    // frequency-domain AC solver. The steady-state operating-point
                    // currents (Id, Iq) are used as initial conditions to eliminate
                    // the DC transient offset that a cold start (0,0) would produce.
                    solver = new DQTransientSolverStar(dqMotor, uu, uv, uw, time, periods,
                        id_init: op.Id, iq_init: op.Iq);
                    break;
                }

                case SolverType.Transient:
                {
                    // NEW: DQ 2×2 backward Euler — uses dqMotor directly
                    solver = new DQTransientSolverStar(dqMotor, uu, uv, uw, time, periods);
                    break;
                }

                case SolverType.TransientWithLCFilter:
                {
                    // LEGACY: ABC 6-state coupled LC+motor (backward compatibility)
                    if (pwmU == null)
                        throw new InvalidOperationException(
                            "TransientWithLCFilter requires LC filter to be active.");
                    var outputLcFilter = (OutputLCFilter)pwm;
                    var transientLcSolver = new TransientSolverWithLCFilter(
                        abcMotor, pwmU, pwmV, pwmW, time, periods,
                        outputLcFilter.Lf, outputLcFilter.Cf);
                    uu = transientLcSolver.MotorVoltageU;
                    uv = transientLcSolver.MotorVoltageV;
                    uw = transientLcSolver.MotorVoltageW;
                    solver = transientLcSolver;
                    useCoupledLcSolver = true;
                    break;
                }

                case SolverType.TransientWithLCFilterFull:
                {
                    // NEW: DQ 6-state fully coupled LC+motor
                    if (pwmU == null)
                        throw new InvalidOperationException(
                            "TransientWithLCFilterFull requires LC filter to be active.");
                    var lcFilterFull = (OutputLCFilter)pwm;
                    var fullSolver = new DQTransientSolverWithLCFilterFull(
                        dqMotor, pwmU, pwmV, pwmW, time, periods,
                        lcFilterFull.Lf, lcFilterFull.Cf);
                    uu = fullSolver.MotorVoltageU;
                    uv = fullSolver.MotorVoltageV;
                    uw = fullSolver.MotorVoltageW;
                    solver = fullSolver;
                    useCoupledLcSolver = true;
                    break;
                }

                default:
                    throw new ArgumentOutOfRangeException(nameof(solverType),
                        $"Unknown solver type: {solverType}");
            }

            // ---- Step 5: Build result ----
            // When LC filter is active: VU/VV/VW = raw PWM voltage, MotorVU/MotorVV/MotorVW = motor-side voltage
            // When LC filter is NOT active: VU/VV/VW = motor-side voltage (same as raw PWM), Motor fields = null
            var result = new PMSMDriveResult
            {
                OperatingPoint = op,
                Time = time,
                VU = pwmU ?? uu,
                VV = pwmV ?? uv,
                VW = pwmW ?? uw,
                IU = solver.CurrentU,
                IV = solver.CurrentV,
                IW = solver.CurrentW,
                BaseFrequencyHz = freqHz,
            };

            // Fill motor-side voltages only when LC filter is active
            if (pwmU != null)
            {
                result.MotorVU = uu;
                result.MotorVV = uv;
                result.MotorVW = uw;
            }

            // Compute line-to-line voltages: U_UV = V_U - V_V, etc.
            int n = time.Count;
            var rawUU = pwmU ?? uu;
            var rawUV = pwmV ?? uv;
            var rawUW = pwmW ?? uw;
            var vuv = new List<double>(n);
            var vvw = new List<double>(n);
            var vwu = new List<double>(n);
            for (int i = 0; i < n; i++)
            {
                vuv.Add(rawUU[i] - rawUV[i]);
                vvw.Add(rawUV[i] - rawUW[i]);
                vwu.Add(rawUW[i] - rawUU[i]);
            }
            result.VUV = vuv;
            result.VVW = vvw;
            result.VWU = vwu;

            // Compute motor-side line-to-line voltages if LC filter is active
            if (pwmU != null)
            {
                var motorVuv = new List<double>(n);
                var motorVvw = new List<double>(n);
                var motorVwu = new List<double>(n);
                for (int i = 0; i < n; i++)
                {
                    motorVuv.Add(uu[i] - uv[i]);
                    motorVvw.Add(uv[i] - uw[i]);
                    motorVwu.Add(uw[i] - uu[i]);
                }
                result.MotorVUV = motorVuv;
                result.MotorVVW = motorVvw;
                result.MotorVWU = motorVwu;
            }

            // Compute FFTs for voltages and currents
            // FFT window logic: if periods < 12, use last 1 period; if >= 12, use last 4 periods
            int fftPeriods = periods < 12 ? 1 : 4;
            int fftLength = n * fftPeriods / Math.Max(periods, 1);
            if (fftLength < 4) fftLength = Math.Min(n, 4); // ensure minimum meaningful FFT size
            int fftStart = n - fftLength;

            if (uu.Count > 0)
            {
                result.VU_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(rawUU, fftStart, fftLength)), fftPeriods);
                result.VV_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(rawUV, fftStart, fftLength)), fftPeriods);
                result.VW_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(rawUW, fftStart, fftLength)), fftPeriods);
            }
            if (vuv.Count > 0)
            {
                result.VUV_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(vuv, fftStart, fftLength)), fftPeriods);
                result.VVW_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(vvw, fftStart, fftLength)), fftPeriods);
                result.VWU_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(vwu, fftStart, fftLength)), fftPeriods);
            }
            if (pwmU != null && uu.Count > 0)
            {
                if (useCoupledLcSolver)
                {
                    // Motor voltages are already solved in time domain by the coupled
                    // LC-filter solver. FFT them directly (same approach as current FFTs).
                    result.MotorVU_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(uu, fftStart, fftLength)), fftPeriods);
                    result.MotorVV_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(uv, fftStart, fftLength)), fftPeriods);
                    result.MotorVW_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(uw, fftStart, fftLength)), fftPeriods);
                }
                else
                {
                    // Use frequency-domain filtering to avoid double-FFT spectral leakage.
                    // Instead of FFT'ing the filtered time-domain signal (which was reconstructed
                    // from only topK harmonics in ApplyFilter, causing spectral leakage), apply the
                    // LC filter transfer function H(jω) directly to the raw PWM FFT.
                    var filter = (OutputLCFilter)pwm;
                    var rawUfft = FFTOperations.GetFFT(ExtractWindow(rawUU, fftStart, fftLength));
                    var rawVfft = FFTOperations.GetFFT(ExtractWindow(rawUV, fftStart, fftLength));
                    var rawWfft = FFTOperations.GetFFT(ExtractWindow(rawUW, fftStart, fftLength));

                    result.MotorVU_FFT = ScaleFFTOrders(filter.GetOutputVoltageFFT(rawUfft, freqHz, 0.0, fftPeriods), fftPeriods);
                    result.MotorVV_FFT = ScaleFFTOrders(filter.GetOutputVoltageFFT(rawVfft, freqHz, -2.0 * Math.PI / 3.0, fftPeriods), fftPeriods);
                    result.MotorVW_FFT = ScaleFFTOrders(filter.GetOutputVoltageFFT(rawWfft, freqHz, -4.0 * Math.PI / 3.0, fftPeriods), fftPeriods);
                }

                // Compute line-to-line motor voltage FFTs from phase FFTs in frequency domain
                result.MotorVUV_FFT = SubtractFFT(result.MotorVU_FFT, result.MotorVV_FFT);
                result.MotorVVW_FFT = SubtractFFT(result.MotorVV_FFT, result.MotorVW_FFT);
                result.MotorVWU_FFT = SubtractFFT(result.MotorVW_FFT, result.MotorVU_FFT);
            }
            if (solver.CurrentU.Count > 0)
            {
                result.IU_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(solver.CurrentU, fftStart, fftLength)), fftPeriods);
                result.IV_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(solver.CurrentV, fftStart, fftLength)), fftPeriods);
                result.IW_FFT = ScaleFFTOrders(FFTOperations.GetFFT(ExtractWindow(solver.CurrentW, fftStart, fftLength)), fftPeriods);
            }

            // ---- Step 6: Fundamental voltage computation ----
            // Extract actual PWM line-to-line fundamental from FFT of VUV (order=1).
            // This is the peak fundamental line-to-line voltage from the actual PWM waveform,
            // which may differ from the target when overmodulation occurs.
            double pwmFundamentalVll;
            if (result.VUV_FFT.Amplitude != null && result.VUV_FFT.Amplitude.Count > 1)
            {
                // Order=1 amplitude = peak line-to-line fundamental voltage
                pwmFundamentalVll = result.VUV_FFT.Amplitude[1];
            }
            else
            {
                // Fallback to ideal computation if FFT not available
                pwmFundamentalVll = Math.Sqrt(3.0) * Math.Sqrt(Ud * Ud + Uq * Uq) / Math.Sqrt(2.0);
            }
            result.PwmVPhasePhaseFundamental = pwmFundamentalVll;

            // Motor-side voltages only when LC filter is active
            if (pwmU != null)
            {
                // Motor phase-to-neutral RMS: average RMS of motor phase voltages
                double rmsMotorVu = ComputeRMS(uu);
                double rmsMotorVv = ComputeRMS(uv);
                double rmsMotorVw = ComputeRMS(uw);
                result.MotorVPhaseNeutralRms = (rmsMotorVu + rmsMotorVv + rmsMotorVw) / 3.0;
            }

            // ---- Step 7: Recompute operating point using synchronous-demodulated currents ----
            // The PWM output fundamental may differ from the target due to overmodulation or LC filtering.
            // Motor currents are taken from the solver's synchronous demodulation (DC-mean of DQ states
            // over the steady-state window), which is immune to FFT spectral leakage from switching
            // harmonics.  Voltages are still derived from FFT fundamental (accurate for PWM voltage).
            double omega = 2.0 * Math.PI * freqHz;

            if (pwmU != null)
            {
                // ── LC filter active ──
                var actualLcFilter = (OutputLCFilter)pwm;
                double vDqActual = pwmFundamentalVll / Math.Sqrt(3.0);
                double vAngle = op.VoltageAngleRad;
                double inverterUd = vDqActual * Math.Cos(vAngle);
                double inverterUq = vDqActual * Math.Sin(vAngle);

                var (motorUd, motorUq) = actualLcFilter.ComputeMotorDQVoltage(inverterUd, inverterUq, omega, abcMotor.PMFluxLinkage);
                result.MotorVPhasePhaseFundamental = Math.Sqrt(3.0) * Math.Sqrt(motorUd * motorUd + motorUq * motorUq) / Math.Sqrt(2.0);

                // Use synchronous-demodulated currents from the solver when available,
                // fall back to steady-state solver otherwise.
                double idActual, iqActual, currentMagnitude, currentAngleRad, torqueNm;
                if (solver.IdFund.HasValue && solver.IqFund.HasValue)
                {
                    idActual = solver.IdFund.Value;
                    iqActual = solver.IqFund.Value;
                    currentMagnitude = Math.Sqrt(idActual * idActual + iqActual * iqActual);
                    currentAngleRad = Math.Atan2(iqActual, idActual);
                    torqueNm = 1.5 * (op.Poles / 2.0) * (op.PMFluxLinkage * iqActual + (op.Ld - op.Lq) * idActual * iqActual);
                }
                else
                {
                    var motorOp = SteadyState.ComputeOperatingPoint(motorUd, motorUq);
                    idActual = motorOp.Id;
                    iqActual = motorOp.Iq;
                    currentMagnitude = motorOp.CurrentMagnitude;
                    currentAngleRad = motorOp.CurrentAngleRad;
                    torqueNm = motorOp.TorqueNm;
                }

                double vMotorMag = Math.Sqrt(motorUd * motorUd + motorUq * motorUq);
                double vInvMag = Math.Sqrt(inverterUd * inverterUd + inverterUq * inverterUq);
                double apparentPowerVA = 1.5 * vMotorMag * currentMagnitude;
                double pwmApparentPower = 1.5 * vInvMag * currentMagnitude;
                double omegaMech = omega / (op.Poles / 2.0);
                double mechanicalPowerW = torqueNm * omegaMech;
                double copperLossW = 1.5 * op.PhaseResistance * (idActual * idActual + iqActual * iqActual);
                double activePowerW = mechanicalPowerW + copperLossW;
                double powerFactor = Math.Abs(activePowerW) / Math.Max(apparentPowerVA, 1e-12);

                result.OperatingPoint = new OperatingPointResult
                {
                    Ud = op.Ud,
                    Uq = op.Uq,
                    Id = idActual,
                    Iq = iqActual,
                    TorqueNm = torqueNm,
                    PhaseVoltageMagnitude = op.PhaseVoltageMagnitude,
                    VoltageAngleRad = op.VoltageAngleRad,
                    CurrentMagnitude = currentMagnitude,
                    CurrentAngleRad = currentAngleRad,
                    ApparentPowerVA = apparentPowerVA,
                    PwmApparentPowerVA = pwmApparentPower,
                    ActivePowerW = activePowerW,
                    PowerFactor = powerFactor,
                    MechanicalPowerW = mechanicalPowerW,
                    CopperLossW = copperLossW,
                    MotorUd = motorUd,
                    MotorUq = motorUq,
                    ElectricalSpeedRadS = op.ElectricalSpeedRadS,
                    ElectricalFreqHz = op.ElectricalFreqHz,
                    SpeedRPM = op.SpeedRPM,
                    Poles = op.Poles,
                    PhaseResistance = op.PhaseResistance,
                    Ld = op.Ld,
                    Lq = op.Lq,
                    PMFluxLinkage = op.PMFluxLinkage,
                };
            }
            else
            {
                // ── No LC filter ──
                if (pwmFundamentalVll > 0)
                {
                    double vDqActual = pwmFundamentalVll / Math.Sqrt(3.0);
                    double vAngle = op.VoltageAngleRad;
                    double actualUd = vDqActual * Math.Cos(vAngle);
                    double actualUq = vDqActual * Math.Sin(vAngle);

                    double idActual, iqActual, currentMagnitude, currentAngleRad, torqueNm;
                    if (solver.IdFund.HasValue && solver.IqFund.HasValue)
                    {
                        idActual = solver.IdFund.Value;
                        iqActual = solver.IqFund.Value;
                        currentMagnitude = Math.Sqrt(idActual * idActual + iqActual * iqActual);
                        currentAngleRad = Math.Atan2(iqActual, idActual);
                        torqueNm = 1.5 * (op.Poles / 2.0) * (op.PMFluxLinkage * iqActual + (op.Ld - op.Lq) * idActual * iqActual);
                    }
                    else
                    {
                        var actualOp = SteadyState.ComputeOperatingPoint(actualUd, actualUq);
                        idActual = actualOp.Id;
                        iqActual = actualOp.Iq;
                        currentMagnitude = actualOp.CurrentMagnitude;
                        currentAngleRad = actualOp.CurrentAngleRad;
                        torqueNm = actualOp.TorqueNm;
                    }

                    double vMotorMag = Math.Sqrt(actualUd * actualUd + actualUq * actualUq);
                    double vInvMag = Math.Sqrt(Ud * Ud + Uq * Uq);
                    double apparentPowerVA = 1.5 * vMotorMag * currentMagnitude;
                    double pwmApparentPower = 1.5 * vInvMag * currentMagnitude;
                    double omegaMech2 = omega / (op.Poles / 2.0);
                    double mechanicalPowerW2 = torqueNm * omegaMech2;
                    double copperLossW2 = 1.5 * op.PhaseResistance * (idActual * idActual + iqActual * iqActual);
                    double activePowerW2 = mechanicalPowerW2 + copperLossW2;
                    double powerFactor2 = Math.Abs(activePowerW2) / Math.Max(apparentPowerVA, 1e-12);

                    result.OperatingPoint = new OperatingPointResult
                    {
                        Ud = op.Ud,
                        Uq = op.Uq,
                        Id = idActual,
                        Iq = iqActual,
                        TorqueNm = torqueNm,
                        PhaseVoltageMagnitude = op.PhaseVoltageMagnitude,
                        VoltageAngleRad = op.VoltageAngleRad,
                        CurrentMagnitude = currentMagnitude,
                        CurrentAngleRad = currentAngleRad,
                        ApparentPowerVA = apparentPowerVA,
                        PwmApparentPowerVA = pwmApparentPower,
                        ActivePowerW = activePowerW2,
                        PowerFactor = powerFactor2,
                        MechanicalPowerW = mechanicalPowerW2,
                        CopperLossW = copperLossW2,
                        MotorUd = actualUd,
                        MotorUq = actualUq,
                        ElectricalSpeedRadS = op.ElectricalSpeedRadS,
                        ElectricalFreqHz = op.ElectricalFreqHz,
                        SpeedRPM = op.SpeedRPM,
                        Poles = op.Poles,
                        PhaseResistance = op.PhaseResistance,
                        Ld = op.Ld,
                        Lq = op.Lq,
                        PMFluxLinkage = op.PMFluxLinkage,
                    };
                }
            }

            return result;
        }

        /// <summary>
        /// Quick access: compute operating point only (no PWM/simulation).
        /// Same as SteadyState.ComputeOperatingPoint(Ud, Uq).
        /// </summary>
        public OperatingPointResult ComputeOperatingPoint(double Ud, double Uq)
        {
            return SteadyState.ComputeOperatingPoint(Ud, Uq);
        }

        /// <summary>
        /// Quick access: get phasor diagram data.
        /// </summary>
        public PhasorDiagramData GetPhasorDiagram(double Ud, double Uq)
        {
            return SteadyState.GetPhasorDiagram(Ud, Uq);
        }

        /// <summary>
        /// Extract a window of samples from a list for FFT analysis.
        /// Returns the last <paramref name="length"/> elements starting at <paramref name="startIndex"/>.
        /// </summary>
        private static double[] ExtractWindow(List<double> signal, int startIndex, int length)
        {
            if (signal == null || signal.Count == 0)
                return [];
            if (startIndex < 0) startIndex = 0;
            if (startIndex + length > signal.Count)
                length = signal.Count - startIndex;
            if (length <= 0)
                return [];

            double[] window = new double[length];
            signal.CopyTo(startIndex, window, 0, length);
            return window;
        }

        /// <summary>
        /// Subtract two FFTContainers element-by-order: result[k] = a[k] - b[k] in frequency domain.
        /// Both FFTs must have the same orders (same-length FFT windows).
        /// Used to compute line-to-line voltage FFTs from phase voltage FFTs.
        /// </summary>
        private static FFTContainer SubtractFFT(FFTContainer a, FFTContainer b)
        {
            int n = Math.Min(a.Amplitude.Count, b.Amplitude.Count);
            var orders = new List<int>(n);
            var amps = new List<double>(n);
            var phases = new List<double>(n);

            for (int i = 0; i < n; i++)
            {
                // Convert amplitude/phase (degrees) to complex, subtract, convert back
                double aRad = a.Phase[i] * Math.PI / 180.0;
                double bRad = b.Phase[i] * Math.PI / 180.0;
                Complex va = Complex.FromPolarCoordinates(a.Amplitude[i], aRad);
                Complex vb = Complex.FromPolarCoordinates(b.Amplitude[i], bRad);
                Complex vdiff = va - vb;

                orders.Add(a.Order[i]);
                amps.Add(vdiff.Magnitude);
                phases.Add(vdiff.Phase * 180.0 / Math.PI); // radians → degrees
            }

            return new FFTContainer
            {
                Order = orders,
                Amplitude = amps,
                Phase = phases
            };
        }

        /// <summary>
        /// Compute RMS (Root Mean Square) of a time-domain signal.
        /// RMS = sqrt( (1/N) * sum(x[i]²) )
        /// </summary>
        private static double ComputeRMS(List<double> x)
        {
            if (x == null || x.Count == 0)
                return 0.0;
            double sumSq = 0.0;
            int n = x.Count;
            for (int i = 0; i < n; i++)
            {
                double val = x[i];
                sumSq += val * val;
            }
            return Math.Sqrt(sumSq / n);
        }

        /// <summary>
        /// Scale FFT order values by dividing by the number of periods in the window,
        /// keeping only bins that align to integer harmonic orders.
        /// When FFT runs on a window of N periods, bin N corresponds to the fundamental.
        /// Dividing by N maps bin N → order 1, bin 2N → order 2, etc.
        /// </summary>
        private static FFTContainer ScaleFFTOrders(FFTContainer fft, int fftPeriods)
        {
            if (fft.Amplitude == null || fft.Amplitude.Count == 0 || fftPeriods <= 1)
                return fft;

            int count = fft.Order.Count;
            var newOrder = new List<int>();
            var newAmplitude = new List<double>();
            var newPhase = new List<double>();

            for (int i = 0; i < count; i++)
            {
                int bin = fft.Order[i];
                // Only keep bins that are exact integer multiples of fftPeriods
                if (bin % fftPeriods == 0)
                {
                    newOrder.Add(bin / fftPeriods);
                    newAmplitude.Add(fft.Amplitude[i]);
                    newPhase.Add(fft.Phase[i]);
                }
            }

            return new FFTContainer
            {
                Order = newOrder,
                Amplitude = newAmplitude,
                Phase = newPhase
            };
        }
    }
}
