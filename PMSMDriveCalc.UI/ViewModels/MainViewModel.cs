using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
using Avalonia.Media;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using PMSMDriveCalc;

namespace PMSMDriveCalc.UI.ViewModels;

public partial class MainViewModel : ObservableObject
{
    // ── Motor parameters ──
    [ObservableProperty] private double _poles = 8;
    [ObservableProperty] private double _phaseResistance = 0.15;
    [ObservableProperty] private double _ld = 0.0025;
    [ObservableProperty] private double _lq = 0.005;
    [ObservableProperty] private double _pMFluxLinkage = 0.125;
    [ObservableProperty] private double _speedRpm = 3000;

    // ── Inverter inputs (line-to-line amplitude "peak" voltage + phase angle) ──
    // TargetVoltage = V_LL_peak = √3 × V_PN_peak. For defaults Ud=100, Uq=150: V_PN_peak ≈ 180.28, V_LL_peak ≈ 312.25
    [ObservableProperty] private double _targetVoltage = 400;
    [ObservableProperty] private double _phaseAngleDeg = 151.5;

    // Computed DQ voltages (derived from polar coordinates, V_PN_peak = V_LL_peak / √3)
    public double Ud => TargetVoltage / Math.Sqrt(3.0) * Math.Cos(PhaseAngleDeg * Math.PI / 180.0);
    public double Uq => TargetVoltage / Math.Sqrt(3.0) * Math.Sin(PhaseAngleDeg * Math.PI / 180.0);

    // ── PWM Settings ──
    [ObservableProperty] private double _switchingFrequency = 8000;
    [ObservableProperty] private double _dCLink = 400;

    [ObservableProperty] private int _modulationIndex = 2; // default: SVPWM2
    public string[] ModulationTypes { get; } = ["SPWM2", "IPSPWM3", "SVPWM2", "SVPWM3"];

    [ObservableProperty] private int _thirdHarmonicInjection; // 0=Off, 1=On

    /// <summary>3rd harmonic injection is only meaningful for SPWM2 (index 0) and IPSPWM3 (index 1).</summary>
    public bool IsThirdHarmonicEnabled => ModulationIndex == 0 || ModulationIndex == 1;

    // ── Output filter (LC filter between inverter & motor) ──
    [ObservableProperty] private bool _enableOutputLCFilter;
    [ObservableProperty] private double _filterInductance = 0.0004;       // Lf (H)
    [ObservableProperty] private double _filterCapacitance = 0.000025;     // Cf (F), Y-connected per-phase
    // MotorInductanceEq is derived from Ld, Lq — removed manual input

    [ObservableProperty] private double _lSigmaRatio = 0.1;               // Leakage/d-axis inductance ratio (0.0–0.5)


    // ── Grid / DC-link ripple filter ──
    [ObservableProperty] private bool _enableGridFilter;
    [ObservableProperty] private double _gridVoltageAmplitude = 245;      // Vp (≈ 230Vrms * √2)
    [ObservableProperty] private double _gridFrequency = 50;               // Hz
    [ObservableProperty] private double _dcLinkCapacitance = 0.0035;       // C_dc (F)
    [ObservableProperty] private double _dcLinkInductance = 0.0005;                 // L_dc (H)
    [ObservableProperty] private double _dcLoadCurrent;                   // Idc (A) – calculated automatically

    // ── Solver Settings ──
    [ObservableProperty] private int _solverTypeIndex = 0; // Without LC: 0=Transient(DQ). With LC: 0=Full-Transient
    public string[] SolverTypes => EnableOutputLCFilter
        ? ["Full-Transient (DQ 6-State)"]
        : ["Transient (DQ)"];

    public string ApparentPowerLabel => EnableOutputLCFilter
        ? "Inv. Apparent Power (VA)"
        : "Apparent Power (VA)";

    partial void OnEnableOutputLCFilterChanged(bool value)
    {
        OnPropertyChanged(nameof(SolverTypes));
        OnPropertyChanged(nameof(ApparentPowerLabel));

        // Clamp solver type index to valid range for the new filter state.
        // Without LC: only "Transient (DQ)" at index 0.
        // With LC: "Full-Transient (DQ 6-State)" at index 0.
        SolverTypeIndex = 0;
    }

    [ObservableProperty] private int _periods = 20;

    // ── Results ──
    [ObservableProperty] private string _statusMessage = "Ready. Enter parameters and click Calculate!.";
    [ObservableProperty] private OperatingPointResult? _operatingPoint;
    [ObservableProperty] private PMSMDriveResult? _driveResult;

    [ObservableProperty] private bool _isComputing;

    [ObservableProperty] private bool _isDeviationSignificant;
    public IBrush ActualForeground => IsDeviationSignificant
        ? new SolidColorBrush(Color.FromRgb(230, 126, 34))   // #E67E22 orange
        : new SolidColorBrush(Color.FromRgb(17, 24, 39));    // #111827 dark

    partial void OnIsDeviationSignificantChanged(bool value)
    {
        OnPropertyChanged(nameof(ActualForeground));
    }

    [ObservableProperty] private double _iIdealVoltageMagnitude;
    [ObservableProperty] private double _opVoltageMagnitude;

    // Flat display properties — "Idealized" (steady-state from target Ud/Uq, no PWM)
    [ObservableProperty] private double _iIdealId;
    [ObservableProperty] private double _iIdealIq;
    [ObservableProperty] private double _iIdealUd;
    [ObservableProperty] private double _iIdealUq;
    [ObservableProperty] private double _iIdealTorqueNm;
    [ObservableProperty] private double _iIdealActivePowerW;
    [ObservableProperty] private double _iIdealApparentPowerVA;
    [ObservableProperty] private double _iIdealPowerFactor;
    [ObservableProperty] private double _iIdealCurrentMagnitude;
    [ObservableProperty] private double _iIdealMechanicalPowerW;
    [ObservableProperty] private double _iIdealCopperLossW;
    [ObservableProperty] private double _iIdealElectricalFreqHz;

    // Flat display properties — "Actual" (from FFT-evaluated fundamentals of PWM output)
    [ObservableProperty] private double _opId;
    [ObservableProperty] private double _opIq;
    [ObservableProperty] private double _opUd;
    [ObservableProperty] private double _opUq;
    [ObservableProperty] private double _opTorqueNm;
    [ObservableProperty] private double _opActivePowerW;
    [ObservableProperty] private double _opPhaseVoltageMagnitude;
    [ObservableProperty] private double _opVoltageAngleRad;
    [ObservableProperty] private double _opCurrentMagnitude;
    [ObservableProperty] private double _opCurrentAngleRad;
    [ObservableProperty] private double _opApparentPowerVA;
    [ObservableProperty] private double _opPwmApparentPowerVA;
    [ObservableProperty] private double _opPowerFactor;
    [ObservableProperty] private double _opElectricalFreqHz;
    [ObservableProperty] private double _opMechanicalPowerW;
    [ObservableProperty] private double _opCopperLossW;

    // PWM / Motor voltage
    [ObservableProperty] private double _pwmVPhasePhaseFundamental;
    [ObservableProperty] private double _motorVPhaseNeutralRms;
    [ObservableProperty] private double _motorVPhasePhaseFundamental;

    partial void OnTargetVoltageChanged(double value)
    {
        var clamped = ClampTargetVoltage(value);
        if (clamped < value)
        {
            TargetVoltage = clamped;
            return; // Setting TargetVoltage re-triggers this method with the clamped value
        }
        OnPropertyChanged(nameof(Ud));
        OnPropertyChanged(nameof(Uq));
    }

    partial void OnPhaseAngleDegChanged(double value)
    {
        OnPropertyChanged(nameof(Ud));
        OnPropertyChanged(nameof(Uq));
    }

    private double ClampTargetVoltage(double value)
    {
        if (ModulationIndex == 2 || ModulationIndex == 3)
            return Math.Min(value, DCLink);
        return value;
    }

    partial void OnDCLinkChanged(double value)
    {
        if ((ModulationIndex == 2 || ModulationIndex == 3) && TargetVoltage > value)
            TargetVoltage = value;
    }

    partial void OnModulationIndexChanged(int value)
    {
        if ((value == 2 || value == 3) && TargetVoltage > DCLink)
            TargetVoltage = DCLink;
        OnPropertyChanged(nameof(IsThirdHarmonicEnabled));
        // Reset 3rd harmonic to Off when switching to a modulation that doesn't support it
        if (!IsThirdHarmonicEnabled)
            ThirdHarmonicInjection = 0;
    }

    partial void OnOperatingPointChanged(OperatingPointResult? value)
    {
        if (value.HasValue)
        {
            var o = value.Value;
            OpId = o.Id;
            OpIq = o.Iq;
            OpUd = o.MotorUd;
            OpUq = o.MotorUq;
            OpTorqueNm = o.TorqueNm;
            OpActivePowerW = o.ActivePowerW;
            OpPhaseVoltageMagnitude = o.PhaseVoltageMagnitude;
            OpVoltageAngleRad = o.VoltageAngleRad;
            OpCurrentMagnitude = o.CurrentMagnitude;
            OpCurrentAngleRad = o.CurrentAngleRad;
            OpApparentPowerVA = o.ApparentPowerVA;
            OpPwmApparentPowerVA = o.PwmApparentPowerVA;
            OpPowerFactor = o.PowerFactor;
            OpElectricalFreqHz = o.ElectricalFreqHz;
            OpMechanicalPowerW = o.MechanicalPowerW;
            OpCopperLossW = o.CopperLossW;
        }
    }

    // Phasor data — Actual (from FFT-derived motor terminal voltages)
    public ObservableCollection<PhasorItem> PhasorItems { get; } = [];
    // Phasor data — Idealized (from target Ud/Uq)
    public ObservableCollection<PhasorItem> PhasorItemsIdealized { get; } = [];

    // ── Plot data accessors ──
    public double[] TimeData => DriveResult?.Time?.ToArray() ?? [];
    public double[] VU => DriveResult?.VU?.ToArray() ?? [];
    public double[] VV => DriveResult?.VV?.ToArray() ?? [];
    public double[] VW => DriveResult?.VW?.ToArray() ?? [];
    public double[] IU => DriveResult?.IU?.ToArray() ?? [];
    public double[] IV => DriveResult?.IV?.ToArray() ?? [];
    public double[] IW => DriveResult?.IW?.ToArray() ?? [];

    // Line-to-line voltages (for PWM Voltages tab display)
    public double[] VUV => DriveResult?.VUV?.ToArray() ?? [];
    public double[] VVW => DriveResult?.VVW?.ToArray() ?? [];
    public double[] VWU => DriveResult?.VWU?.ToArray() ?? [];

    // Motor-side line-to-line voltages (for Motor Voltage tab, only populated when LC filter active)
    public double[] MotorVUV => DriveResult?.MotorVUV?.ToArray() ?? [];
    public double[] MotorVVW => DriveResult?.MotorVVW?.ToArray() ?? [];
    public double[] MotorVWU => DriveResult?.MotorVWU?.ToArray() ?? [];

    public FFTContainer? VU_FFT => DriveResult?.VU_FFT;
    public FFTContainer? VV_FFT => DriveResult?.VV_FFT;
    public FFTContainer? VW_FFT => DriveResult?.VW_FFT;
    public FFTContainer? VUV_FFT => DriveResult?.VUV_FFT;
    public FFTContainer? VVW_FFT => DriveResult?.VVW_FFT;
    public FFTContainer? VWU_FFT => DriveResult?.VWU_FFT;
    public FFTContainer? MotorVUV_FFT => DriveResult?.MotorVUV_FFT;
    public FFTContainer? MotorVVW_FFT => DriveResult?.MotorVVW_FFT;
    public FFTContainer? MotorVWU_FFT => DriveResult?.MotorVWU_FFT;
    public FFTContainer? IU_FFT => DriveResult?.IU_FFT;
    public FFTContainer? IV_FFT => DriveResult?.IV_FFT;
    public FFTContainer? IW_FFT => DriveResult?.IW_FFT;

    public static double[] GetFftMagnitude(FFTContainer? fft)
    {
        if (!fft.HasValue || fft.Value.Amplitude == null) return [];
        return [.. fft.Value.Amplitude];
    }

    // ── Commands ──
    [RelayCommand]
    private async Task ComputeAll()
    {
        IsComputing = true;
        StatusMessage = "Calculating...";
        try
        {
            var motor = CreateMotor();

            ICanOutputVoltage pwm = ModulationIndex switch
            {
                1 => new IPSPWM3(SwitchingFrequency, DCLink, ThirdHarmonicInjection),
                2 => new QuasiSVPWM2(SwitchingFrequency, DCLink),
                3 => new QuasiSVPWM3(SwitchingFrequency, DCLink),
                _ => new SPWM2(SwitchingFrequency, DCLink, ThirdHarmonicInjection)
            };

            // ── Wire output filters ──
            ICanOutputVoltage finalOutput = pwm;

            // 1. Grid-side DC-link ripple filter (wraps PWM)
            if (EnableGridFilter)
            {
                var grid = new GridRectifierFilter(GridVoltageAmplitude, GridFrequency,
                    DcLinkCapacitance, DcLinkInductance);
                // Estimate load current from nominal motor power
                double estPower = OperatingPoint?.ActivePowerW ?? 5000;
                grid.SetLoadFromMotorPower(estPower);
                DcLoadCurrent = grid.AverageDCCurrent;
                finalOutput = new DCFilteredPWM(pwm, grid, DCLink);
            }

            // 2. Motor-side LC output filter (wraps DCFilteredPWM or raw PWM)
            if (EnableOutputLCFilter)
            {
                // Leq = (Ld + Lq) / 2 for Y-connected PMSM (effective fundamental inductance)
                double leq = (Ld + Lq) / 2.0;
                // L_sigma = LSigmaRatio * Ld (leakage inductance for high-frequency harmonics, see PMSMdq.LSigma)
                double lSigma = LSigmaRatio * Ld;
                finalOutput = new OutputLCFilter(finalOutput, FilterInductance,
                    FilterCapacitance, PhaseResistance, leq, PMFluxLinkage, lSigma);
            }

            // ── Compute (offload heavy computation to thread pool) ──
            var calc = new PMSMDQDriveCalculator(motor, finalOutput);
            SolverType st;
            if (!EnableOutputLCFilter)
            {
                st = SolverTypeIndex switch
                {
                    0 => SolverType.Transient,     // DQ 2×2 backward Euler
                    _ => SolverType.Transient
                };
            }
            else
            {
                st = SolverType.TransientWithLCFilterFull;   // DQ 6-state fully coupled LC+motor (only option)
            }
            PMSMDriveResult driveResult = null!;
            await Task.Run(() =>
            {
                driveResult = calc.Compute(Ud, Uq, st, Periods);
            });
            DriveResult = driveResult;
            OperatingPoint = driveResult.OperatingPoint;

            // ── "Idealized" operating point ──
            // Start with steady-state from target Ud/Uq (assumes PWM matches target voltage).
            var idealOp = calc.SteadyState.ComputeOperatingPoint(Ud, Uq);

            // When LC output filter is active, account for the filter's effect on
            // motor terminal dq voltage at the fundamental frequency.
            if (finalOutput is OutputLCFilter lcFilter)
            {
                double omega = idealOp.ElectricalSpeedRadS;
                var (motorUd, motorUq) = lcFilter.ComputeMotorDQVoltage(
                    Ud, Uq, omega, PMFluxLinkage);
                idealOp = calc.SteadyState.ComputeOperatingPoint(motorUd, motorUq);
            }

            PopulateIdealized(idealOp);

            // Build idealized phasor diagram from the motor terminal dq voltages
            var idealPd = motor.GetPhasorDiagram(idealOp.MotorUd, idealOp.MotorUq);
            BuildPhasorItemsIdealized(idealPd);

            PwmVPhasePhaseFundamental = driveResult.PwmVPhasePhaseFundamental;
            MotorVPhaseNeutralRms = driveResult.MotorVPhaseNeutralRms ?? 0;
            MotorVPhasePhaseFundamental = driveResult.MotorVPhasePhaseFundamental ?? 0;

            // Build actual phasor diagram (from FFT-derived motor terminal voltages)
            var pd = motor.GetPhasorDiagram(driveResult.OperatingPoint.MotorUd, driveResult.OperatingPoint.MotorUq);
            BuildPhasorItems(pd);

            // Notify all plot data changed
            OnPropertyChanged(nameof(TimeData));
            OnPropertyChanged(nameof(VU));
            OnPropertyChanged(nameof(VV));
            OnPropertyChanged(nameof(VW));
            OnPropertyChanged(nameof(VUV));
            OnPropertyChanged(nameof(VVW));
            OnPropertyChanged(nameof(VWU));
            OnPropertyChanged(nameof(IU));
            OnPropertyChanged(nameof(IV));
            OnPropertyChanged(nameof(IW));
            OnPropertyChanged(nameof(VU_FFT));
            OnPropertyChanged(nameof(VV_FFT));
            OnPropertyChanged(nameof(VW_FFT));
            OnPropertyChanged(nameof(VUV_FFT));
            OnPropertyChanged(nameof(VVW_FFT));
            OnPropertyChanged(nameof(VWU_FFT));
            OnPropertyChanged(nameof(MotorVUV));
            OnPropertyChanged(nameof(MotorVVW));
            OnPropertyChanged(nameof(MotorVWU));
            OnPropertyChanged(nameof(MotorVUV_FFT));
            OnPropertyChanged(nameof(MotorVVW_FFT));
            OnPropertyChanged(nameof(MotorVWU_FFT));
            OnPropertyChanged(nameof(IU_FFT));
            OnPropertyChanged(nameof(IV_FFT));
            OnPropertyChanged(nameof(IW_FFT));
            int pts = TimeData.Length;
            StatusMessage = $"Complete. {pts} time points, " +
                $"Torque={driveResult.OperatingPoint.TorqueNm:F4} Nm" +
                (EnableOutputLCFilter ? " (LC filter active)" : "") +
                (EnableGridFilter ? " (grid ripple active)" : "");
        }
        catch (Exception ex)
        {
            StatusMessage = $"Error: {ex.Message}";
        }
        finally { IsComputing = false; }
    }

    private PMSMdq CreateMotor()
    {
        var motor = new PMSMdq((int)Poles, PMFluxLinkage, Ld, Lq, PhaseResistance, LSigmaRatio);
        motor.SpeedRPM = SpeedRpm;
        return motor;
    }

    private void BuildPhasorItems(PhasorDiagramData pd)
    {
        PhasorItems.Clear();

        void Add(string name, double mag, double phaseRad)
        {
            PhasorItems.Add(new PhasorItem { Name = name, Magnitude = mag, AngleRad = phaseRad });
        }

        // Items 0-1: Current phasors (drawn from origin, dashed)
        Add("Id", pd.Id.mag, pd.Id.phaseRad);
        Add("Iq", pd.Iq.mag, pd.Iq.phaseRad);

        // Items 2-6: Voltage chain (drawn head-to-tail)
        //   Chain: jωψPM → jωLq·Iq → jωLd·Id → R·Id → R·Iq
        //   Tip lands at (Ud, Uq)
        Add("jωψPM", pd.JOmegaPsiPM.mag, pd.JOmegaPsiPM.phaseRad);
        Add("jωLq·Iq", pd.JOmegaLqIq.mag, pd.JOmegaLqIq.phaseRad);
        Add("jωLd·Id", pd.JOmegaLdId.mag, pd.JOmegaLdId.phaseRad);
        Add("R·Id", pd.RId.mag, pd.RId.phaseRad);
        Add("R·Iq", pd.RIq.mag, pd.RIq.phaseRad);

        OnPropertyChanged(nameof(PhasorItems));
    }

    private void BuildPhasorItemsIdealized(PhasorDiagramData pd)
    {
        PhasorItemsIdealized.Clear();

        void Add(string name, double mag, double phaseRad)
        {
            PhasorItemsIdealized.Add(new PhasorItem { Name = name, Magnitude = mag, AngleRad = phaseRad });
        }

        Add("Id", pd.Id.mag, pd.Id.phaseRad);
        Add("Iq", pd.Iq.mag, pd.Iq.phaseRad);
        Add("jωψPM", pd.JOmegaPsiPM.mag, pd.JOmegaPsiPM.phaseRad);
        Add("jωLq·Iq", pd.JOmegaLqIq.mag, pd.JOmegaLqIq.phaseRad);
        Add("jωLd·Id", pd.JOmegaLdId.mag, pd.JOmegaLdId.phaseRad);
        Add("R·Id", pd.RId.mag, pd.RId.phaseRad);
        Add("R·Iq", pd.RIq.mag, pd.RIq.phaseRad);

        OnPropertyChanged(nameof(PhasorItemsIdealized));
    }

    private void PopulateIdealized(OperatingPointResult ideal)
    {
        IIdealId = ideal.Id;
        IIdealIq = ideal.Iq;
        IIdealUd = ideal.Ud;
        IIdealUq = ideal.Uq;
        IIdealTorqueNm = ideal.TorqueNm;
        IIdealActivePowerW = ideal.ActivePowerW;
        IIdealApparentPowerVA = ideal.ApparentPowerVA;
        IIdealPowerFactor = ideal.PowerFactor;
        IIdealCurrentMagnitude = ideal.CurrentMagnitude;
        IIdealMechanicalPowerW = ideal.MechanicalPowerW;
        IIdealCopperLossW = ideal.CopperLossW;
        IIdealElectricalFreqHz = ideal.ElectricalFreqHz;
        IIdealVoltageMagnitude = Math.Sqrt(ideal.Ud * ideal.Ud + ideal.Uq * ideal.Uq);

        // Check deviation after Op* properties are set (they are set via OnOperatingPointChanged)
        OpVoltageMagnitude = Math.Sqrt(OpUd * OpUd + OpUq * OpUq);
        double opMag = OpVoltageMagnitude;
        IsDeviationSignificant = Math.Abs(IIdealVoltageMagnitude - opMag) > 0.8
            || Math.Abs(IIdealUd - OpUd) > 0.8
            || Math.Abs(IIdealUq - OpUq) > 0.8;
    }
}

public class PhasorItem
{
    public string Name { get; set; } = "";
    public double Magnitude { get; set; }
    public double AngleRad { get; set; }
}