using System;
using System.Collections.Generic;

namespace PMSMDriveCalc
{
    public class PMSM
    {
        private int poles;
        private double speedRPM;
        private double angularSpeed;
        private double baseFrequency;
        private double phaseResistance;
        private double pMFluxLinkage;

        public PMSM(int poles, double pMFluxLinkage, double phaseResistance)
        {
            Poles = poles;
            PMFluxLinkage = pMFluxLinkage;
            PhaseResistance = phaseResistance;
        }

        public int Poles { get => poles; set => poles = value; }
        public double PMFluxLinkage { get => pMFluxLinkage; set => pMFluxLinkage = value; }

        public double SpeedRPM
        {
            get => speedRPM;
            set
            {
                speedRPM = value;
                baseFrequency = speedRPM / 60.0 * poles / 2.0;
                angularSpeed = baseFrequency * 2.0 * Math.PI;
            }
        }
        public double AngularSpeed { get => angularSpeed; private set => angularSpeed = value; }
        public double BaseFrequency { get => baseFrequency; private set => baseFrequency = value; }
        public double PhaseResistance { get => phaseResistance; set => phaseResistance = value; }


    }


    /// <summary>
    /// Container for dq-frame phasor diagram data.
    /// Each field holds (magnitude, phase_radians) of the phasor in the dq reference frame,
    /// where the d-axis is the real axis and the q-axis is the imaginary axis.
    /// </summary>
    public struct PhasorDiagramData
    {
        public (double mag, double phaseRad) Ud;
        public (double mag, double phaseRad) Uq;
        public (double mag, double phaseRad) Id;
        public (double mag, double phaseRad) Iq;
        public (double mag, double phaseRad) RId;
        public (double mag, double phaseRad) RIq;
        public (double mag, double phaseRad) JOmegaLdId;
        public (double mag, double phaseRad) JOmegaLqIq;
        public (double mag, double phaseRad) JOmegaPsiPM;
        public double TorqueNm;
    }

    /// <summary>
    /// PMSM model in dq synchronous reference frame.
    /// Supports salient-pole machines (Ld != Lq) and computes electromagnetic torque.
    /// Provides steady-state dq voltage equation solving for given Ud, Uq inputs.
    /// </summary>
    public class PMSMdq : PMSM
    {
        private double Ld;
        private double Lq;
        private double _lSigmaRatio;

        /// <summary>
        /// Create a dq-frame PMSM model.
        /// </summary>
        /// <param name="poles">Number of poles</param>
        /// <param name="pMFluxLinkage">Permanent-magnet flux linkage (Wb)</param>
        /// <param name="Ld">d-axis inductance (H)</param>
        /// <param name="Lq">q-axis inductance (H)</param>
        /// <param name="phaseResistance">Stator phase resistance (Ohm)</param>
        /// <param name="lSigmaRatio">Leakage inductance ratio L_sigma/Ld. Range 0.0–0.5, default 0.1.</param>
        public PMSMdq(int poles, double pMFluxLinkage,
            double Ld, double Lq, double phaseResistance,
            double lSigmaRatio = 0.1)
            : base(poles, pMFluxLinkage, phaseResistance)
        {
            this.Ld = Ld;
            this.Lq = Lq;
            this._lSigmaRatio = Math.Clamp(lSigmaRatio, 0.0, 0.5);
        }

        /// <summary>d-axis inductance (H)</summary>
        public double DInductance { get => Ld; set => Ld = value; }

        /// <summary>q-axis inductance (H)</summary>
        public double QInductance { get => Lq; set => Lq = value; }

        /// <summary>
        /// Leakage inductance ratio (L_sigma / Ld).
        /// Used for LC filter high-frequency impedance calculation.
        /// Range 0.0–0.5, default 0.1.
        /// </summary>
        public double LSigmaRatio
        {
            get => _lSigmaRatio;
            set => _lSigmaRatio = Math.Clamp(value, 0.0, 0.5);
        }

        /// <summary>Leakage inductance L_sigma = ratio · Ld (H).</summary>
        public double LSigma => LSigmaRatio * Ld;

        /// <summary>Effective inductance at fundamental frequency: (Ld + Lq) / 2 (H).</summary>
        public double EffectiveInductance => (Ld + Lq) / 2.0;

        /// <summary>
        /// Electromagnetic torque (Nm) from dq-axis currents.
        /// T_e = 1.5 * (Poles/2) * [ ψ_pm * i_q + (L_d - L_q) * i_d * i_q ]
        /// </summary>
        public double CalculateTorque(double id, double iq)
        {
            double polePairs = Poles / 2.0;
            double relutanceTorque = (Ld - Lq) * id * iq;
            double magnetTorque = PMFluxLinkage * iq;
            return 1.5 * polePairs * (magnetTorque + relutanceTorque);
        }

        /// <summary>
        /// Solve steady-state dq voltage equations for Id, Iq given Ud, Uq.
        /// Equations:
        ///   Ud = R·Id - ω·Lq·Iq
        ///   Uq = R·Iq + ω·Ld·Id + ω·ψ_pm
        /// Matrix form:
        ///   [ R    -ωLq ] [ Id ]   [ Ud          ]
        ///   [ ωLd   R   ] [ Iq ] = [ Uq - ω·ψ_pm ]
        /// </summary>
        /// <param name="Ud">d-axis voltage (V)</param>
        /// <param name="Uq">q-axis voltage (V)</param>
        /// <returns>(Id, Iq) in Amperes</returns>
        public (double Id, double Iq) SolveSteadyStateCurrents(double Ud, double Uq)
        {
            double omega = AngularSpeed;
            double R = PhaseResistance;
            double psiPM = PMFluxLinkage;

            double bd = Ud;
            double bq = Uq - omega * psiPM;

            double det = R * R + omega * omega * Ld * Lq;

            if (Math.Abs(det) < 1e-30)
            {
                // Degenerate case (zero speed): purely resistive
                return (Ud / R, (Uq - omega * psiPM) / R);
            }

            double Id = (R * bd + omega * Lq * bq) / det;
            double Iq = (R * bq - omega * Ld * bd) / det;

            return (Id, Iq);
        }

        /// <summary>
        /// Compute steady-state electromagnetic torque directly from Ud, Uq.
        /// This is the operating-point torque under dq excitation.
        /// </summary>
        public double CalculateSteadyStateTorque(double Ud, double Uq)
        {
            var (Id, Iq) = SolveSteadyStateCurrents(Ud, Uq);
            return CalculateTorque(Id, Iq);
        }

        /// <summary>
        /// Generate complete phasor diagram data for the given dq operating point.
        /// All phasors are referenced to the d-axis (real axis).
        /// </summary>
        /// <param name="Ud">d-axis voltage (V)</param>
        /// <param name="Uq">q-axis voltage (V)</param>
        /// <returns>PhasorDiagramData with magnitudes, phases, and torque</returns>
        public PhasorDiagramData GetPhasorDiagram(double Ud, double Uq)
        {
            double omega = AngularSpeed;
            double R = PhaseResistance;
            double psiPM = PMFluxLinkage;

            var (Id, Iq) = SolveSteadyStateCurrents(Ud, Uq);

            // Voltage drops in dq
            double V_RId = R * Id;
            double V_RIq = R * Iq;
            double V_JOmegaLdId = omega * Ld * Id;
            double V_JOmegaLqIq = omega * Lq * Iq;
            double V_JOmegaPsiPM = omega * psiPM;

            PhasorDiagramData data = new PhasorDiagramData();

            // Ud is purely on d-axis (real)
            data.Ud = (Ud, 0.0);
            // Uq is purely on q-axis (imaginary, +90°)
            data.Uq = (Uq, Math.PI / 2.0);

            // Id is on d-axis (Id > 0 means aligned with +d)
            data.Id = (Math.Abs(Id), Id >= 0 ? 0.0 : Math.PI);
            // Iq is on q-axis (Iq > 0 means aligned with +q)
            data.Iq = (Math.Abs(Iq), Iq >= 0 ? Math.PI / 2.0 : -Math.PI / 2.0);

            // Resistive drops are in phase with their respective currents
            data.RId = (Math.Abs(V_RId), V_RId >= 0 ? 0.0 : Math.PI);
            data.RIq = (Math.Abs(V_RIq), V_RIq >= 0 ? Math.PI / 2.0 : -Math.PI / 2.0);

            // Inductive drops lead by 90° relative to their currents
            // ωLd·Id: Id on d-axis, so jωLd·Id is on +q axis (90°)
            data.JOmegaLdId = (Math.Abs(V_JOmegaLdId), V_JOmegaLdId >= 0 ? Math.PI / 2.0 : -Math.PI / 2.0);
            // ωLq·Iq: Iq on q-axis, so jωLq·Iq is on -d axis (-90° from q = 180° from d)
            data.JOmegaLqIq = (Math.Abs(V_JOmegaLqIq), V_JOmegaLqIq >= 0 ? Math.PI : 0.0);

            // Back EMF ω·ψ_pm is on q-axis
            data.JOmegaPsiPM = (Math.Abs(V_JOmegaPsiPM), V_JOmegaPsiPM >= 0 ? Math.PI / 2.0 : -Math.PI / 2.0);

            data.TorqueNm = CalculateTorque(Id, Iq);

            return data;
        }

        /// <summary>
        /// Get the peak phase voltage magnitude from Ud, Uq.
        /// |U_ph| = sqrt(Ud² + Uq²)
        /// </summary>
        public double GetPhaseVoltageMagnitude(double Ud, double Uq)
        {
            return Math.Sqrt(Ud * Ud + Uq * Uq);
        }

        /// <summary>
        /// Get the voltage phase angle in the dq frame (radians).
        /// Angle = atan2(Uq, Ud) — this is the angle of the voltage vector
        /// relative to the d-axis.
        /// </summary>
        public double GetVoltageAngleDQ(double Ud, double Uq)
        {
            return Math.Atan2(Uq, Ud);
        }
    }


    /// <summary>
    /// [DEPRECATED] ABC-frame motor model with constant self/mutual inductance.
    /// Retained for backward compatibility with legacy ABC-frame solvers and DriveStarConnected.
    /// New code should use PMSMdq with dq-frame transient solvers directly.
    /// </summary>
    [Obsolete("Use PMSMdq with dq-frame transient solvers instead.")]
    public class PMSMWithConstantInductance : PMSM
    {
        private double selfInductance;
        private double mutualInductance;

        public PMSMWithConstantInductance(int poles, double pMFluxLinkage, double selfInductance,
            double mutualInductance, double phaseResistance) : base(poles, pMFluxLinkage, phaseResistance)
        {
            SelfInductance = selfInductance;
            MutualInductance = mutualInductance;
        }

        public double SelfInductance { get => selfInductance; set => selfInductance = value; }
        public double MutualInductance { get => mutualInductance; set => mutualInductance = value; }
    }


    public class PMSMWithVariableInductance : PMSM
    {
        private List<double> selfInductance;
        private List<double> mutualInductance;
        private List<double> referenceFrequency;

        public PMSMWithVariableInductance(int poles, double pMFluxLinkage,
            List<double> selfInductance, List<double> mutualInductance,
            List<double> referenceFrequency, double phaseResistance) : base(poles, pMFluxLinkage, phaseResistance)
        {
            SelfInductance = selfInductance;
            MutualInductance = mutualInductance;
            ReferenceFrequency = referenceFrequency;
        }

        public List<double> SelfInductance { get => selfInductance; set => selfInductance = value; }
        public List<double> MutualInductance { get => mutualInductance; set => mutualInductance = value; }
        public List<double> ReferenceFrequency { get => referenceFrequency; set => referenceFrequency = value; }

        public double[] GetInductanceAtFrequency(double frequency)
        {
            int index1 = 0;
            if (ReferenceFrequency.Count == 1) { return new double[] { SelfInductance[0], MutualInductance[0] }; }
            if (frequency <= ReferenceFrequency[0]) { index1 = 0; }
            else if (frequency >= ReferenceFrequency[ReferenceFrequency.Count - 1])
            {
                index1 = ReferenceFrequency.Count - 2;
            }
            else
            {
                for (int i = 0; i < ReferenceFrequency.Count - 1; i++)
                {
                    if ((frequency > ReferenceFrequency[i]) && (frequency <= ReferenceFrequency[i + 1]))
                    {
                        index1 = i;
                        break;
                    }
                }
            }
            int index2 = index1 + 1;

            double _k = (frequency - ReferenceFrequency[index1]) /
                (ReferenceFrequency[index2] - ReferenceFrequency[index1]);
            double _selfinductance = _k * (SelfInductance[index2] - SelfInductance[index1])
                + SelfInductance[index1];
            double _mutualinductance = _k * (MutualInductance[index2] - MutualInductance[index1])
                + MutualInductance[index1];
            return new double[] { _selfinductance, _mutualinductance };
        }

    }


}
