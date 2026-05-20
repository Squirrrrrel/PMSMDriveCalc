using System;
using System.Collections.Generic;

namespace PMSMDriveCalc
{
    /// <summary>
    /// Decorator that applies dead-time voltage error to any PWM output.
    /// 
    /// Dead-time effect: During the blanking interval (td) where both upper and lower
    /// switches are off, the output voltage is determined by the freewheeling diode
    /// conduction, which depends on the sign of the phase current:
    /// 
    ///   ΔV_err = −sign(i_phase) · (td/Ts) · Vdc
    /// 
    /// This introduces 5th, 7th, 11th, 13th, … characteristic harmonics into the
    /// motor current spectrum — a well-known signature of inverter non-linearity.
    /// 
    /// Current polarity is estimated from the steady-state dq solution at the
    /// reference voltage (one-pass, no iterative solver re-run needed).
    /// 
    /// Usage:
    ///   var pwm = new DeadTimePWM(new QuasiSVPWM2(fsw, vdc), motor, deadTimeUs: 2.0);
    ///   // pwm applies dead-time correction on top of QuasiSVPWM2 switching waveforms
    /// </summary>
    public class DeadTimePWM : ICanOutputVoltage
    {
        private readonly ICanOutputVoltage _innerPWM;
        private readonly PMSMdq _motor;
        private readonly double _deadTime;          // seconds (e.g., 2e-6 for 2 µs)
        private readonly double _vdc;               // DC-link voltage (V)

        /// <param name="innerPWM">PWM strategy to decorate.</param>
        /// <param name="motor">dq-frame motor model for steady-state current estimation.</param>
        /// <param name="deadTimeUs">Dead-time in microseconds (default 2.0 µs).</param>
        public DeadTimePWM(ICanOutputVoltage innerPWM, PMSMdq motor, double deadTimeUs = 2.0)
        {
            _innerPWM = innerPWM ?? throw new ArgumentNullException(nameof(innerPWM));
            _motor = motor ?? throw new ArgumentNullException(nameof(motor));
            _deadTime = deadTimeUs * 1e-6;

            // Extract Vdc from the inner PWM chain
            if (innerPWM is PWM pwmBase)
                _vdc = pwmBase.DCLink;
            else
                _vdc = 400.0; // fallback
        }

        /// <summary>Delegates to the innermost PWM.</summary>
        public double SwitchingFrequency
        {
            get
            {
                if (_innerPWM is PWM pwm)
                    return pwm.SwitchingFrequency;
                return 8000.0;
            }
        }

        /// <summary>
        /// Generate 3-phase switching voltages with dead-time error applied.
        /// 
        /// Algorithm (one-pass):
        /// 1. Get nominal PWM output from inner strategy
        /// 2. Solve steady-state Id, Iq from the reference (Ud, Uq)
        /// 3. For each sample, compute phase current ia[k] = Id·cos(θ) − Iq·sin(θ)
        /// 4. Subtract sign(ia[k]) · (td/Ts) · Vdc from the nominal voltage
        /// 
        /// The correction is applied sample-by-sample. Since sign(i_phase) only changes
        /// near current zero-crossings (narrow window), this is equivalent to the
        /// per-switching-period average dead-time model for most of the waveform.
        /// </summary>
        public List<List<double>> GetOutputVoltage(
            double amplitudeVoltage, double frequencyVoltage,
            double phaseVoltage, int periods)
        {
            // Step 1: Get nominal PWM output from inner
            List<List<double>> nominal = _innerPWM.GetOutputVoltage(
                amplitudeVoltage, frequencyVoltage, phaseVoltage, periods);

            // If dead-time is zero or negligible, return nominal output unchanged
            double Ts = 1.0 / SwitchingFrequency;
            double dV = (_deadTime / Ts) * _vdc;
            if (dV < 1e-9)
                return nominal;

            // Step 2: Compute steady-state Id, Iq from reference voltage
            //   The reference is sin(ωt + phaseVoltage) convention.
            //   In dq cos convention: φ_v = phaseVoltage − π/2
            double omega = 2.0 * Math.PI * frequencyVoltage;
            double phi_v = phaseVoltage - Math.PI / 2.0;
            double Ud = amplitudeVoltage * Math.Cos(phi_v);
            double Uq = amplitudeVoltage * Math.Sin(phi_v);

            // Temporarily set motor speed for the steady-state solve
            double savedRpm = _motor.SpeedRPM;
            double rpmFromFreq = frequencyVoltage * 60.0 / (_motor.Poles / 2.0);
            _motor.SpeedRPM = rpmFromFreq;

            (double Id, double Iq) = _motor.SolveSteadyStateCurrents(Ud, Uq);
            _motor.SpeedRPM = savedRpm;

            // Step 3: Apply dead-time correction to each phase
            List<double> time;
            List<double> vu, vv, vw;

            if (nominal.Count >= 4)
            {
                // 3-phase output (QuasiSVPWM2, QuasiSVPWM3, SVPWM2, SVPWM3, etc.)
                time = nominal[0];
                vu = nominal[1];
                vv = nominal[2];
                vw = nominal[3];
            }
            else
            {
                // Single-phase output (SPWM2, IPSPWM3): generate each phase separately
                time = nominal[0];
                vu = nominal[1];

                var phaseV = _innerPWM.GetOutputVoltage(amplitudeVoltage, frequencyVoltage,
                    phaseVoltage - 2.0 * Math.PI / 3.0, periods);
                var phaseW = _innerPWM.GetOutputVoltage(amplitudeVoltage, frequencyVoltage,
                    phaseVoltage - 4.0 * Math.PI / 3.0, periods);

                vv = phaseV[1];
                vw = phaseW[1];
            }

            int n = time.Count;
            var vuCorrected = new List<double>(n);
            var vvCorrected = new List<double>(n);
            var vwCorrected = new List<double>(n);

            for (int k = 0; k < n; k++)
            {
                double theta = omega * time[k];
                double cosTheta = Math.Cos(theta);
                double sinTheta = Math.Sin(theta);

                // Phase U (phase offset = 0)
                double ia = Id * cosTheta - Iq * sinTheta;
                vuCorrected.Add(vu[k] - Math.Sign(ia) * dV);

                // Phase V (phase offset = −2π/3)
                double thetaV = theta - 2.0 * Math.PI / 3.0;
                double ib = Id * Math.Cos(thetaV) - Iq * Math.Sin(thetaV);
                vvCorrected.Add(vv[k] - Math.Sign(ib) * dV);

                // Phase W (phase offset = −4π/3)
                double thetaW = theta - 4.0 * Math.PI / 3.0;
                double ic = Id * Math.Cos(thetaW) - Iq * Math.Sin(thetaW);
                vwCorrected.Add(vw[k] - Math.Sign(ic) * dV);
            }

            return new List<List<double>> { time, vuCorrected, vvCorrected, vwCorrected };
        }
    }
}
