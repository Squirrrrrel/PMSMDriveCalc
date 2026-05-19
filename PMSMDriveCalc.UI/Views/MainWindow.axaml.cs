using System;
using System.Collections.Generic;
using System.Linq;
using Avalonia.Controls;
using PMSMDriveCalc.UI.ViewModels;
using PMSMDriveCalc;
using ScottPlot;

namespace PMSMDriveCalc.UI.Views;

public partial class MainWindow : Window
{
    private readonly MainViewModel _vm;
    private bool _plotsInitialized;

    public MainWindow()
    {
        InitializeComponent();
        _vm = new MainViewModel();
        DataContext = _vm;

        _vm.PropertyChanged += (s, e) =>
        {
            if (!_plotsInitialized) return;

            if (e.PropertyName == nameof(MainViewModel.PhasorItems))
                RefreshPhasorPlot();
            else if (e.PropertyName == nameof(MainViewModel.PhasorItemsIdealized))
                RefreshPhasorPlotIdealized();
            else if (e.PropertyName == nameof(MainViewModel.TimeData))
                RefreshTimeDomainPlots();
            else if (e.PropertyName == nameof(MainViewModel.MotorVUV_FFT))
                RefreshMotorVoltagePlots();
            else if (e.PropertyName == nameof(MainViewModel.VUV_FFT))
                RefreshFFTPlots();
        };

        Loaded += OnLoaded;
    }

    private void OnLoaded(object? sender, Avalonia.Interactivity.RoutedEventArgs e)
    {
        if (_plotsInitialized) return;
        InitializeAllPlots();
        _plotsInitialized = true;
    }

    private void OnInfoClick(object? sender, Avalonia.Interactivity.RoutedEventArgs e)
    {
        var infoText = new SelectableTextBlock
        {
            Padding = new Avalonia.Thickness(24),
            TextWrapping = Avalonia.Media.TextWrapping.Wrap,
            FontSize = 13,
            Inlines =
            {
                new Avalonia.Controls.Documents.Run("PMSM Drive Calculator")
                {
                    FontSize = 18,
                    FontWeight = Avalonia.Media.FontWeight.Bold,
                },
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.Run(
                    "A design and analysis tool for permanent-magnet synchronous motor (PMSM) drive systems. " +
                    "Built with C# and .NET 9.0 on the Avalonia UI cross-platform framework."),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.Run(
                    "This application models star-connected (Y-connected) PMSM drives supplied by a three-phase " +
                    "symmetrical voltage-source PWM inverter circuit. The motor windings and, when enabled, " +
                    "the LC output filter capacitors are both Y-connected with a floating star point (no neutral return)."),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.Run("📐 Features")
                {
                    FontSize = 14,
                    FontWeight = Avalonia.Media.FontWeight.Bold,
                },
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.Run(
                    "• Steady-state dq-frame motor model (Ld, Lq, flux linkage, resistance)\n" +
                    "• PWM modulation: SPWM, IPSPWM, SVPWM (internally QuasiSVPWM via zero-sequence injection)\n" +
                    "• SVPWM implemented as QuasiSVPWM — mathematically equivalent to traditional space-vector PWM but avoids zero-order-hold (ZOH) phase lag (ω·Ts/2) by evaluating the reference continuously at every sub-sample\n" +
                    "• Leakage Ratio parameter for high-frequency harmonics (0–0.5 range, LC filter)\n" +
                    "• Hover tooltips on all input labels — explanations appear on mouse hover\n" +
                    "• DQ-frame transient solvers with zero-sequence-rejecting Clarke transform\n" +
                    "• LC output filter simulation with coupled DQ 6-state transient solver\n" +
                    "• Grid-side DC-link ripple filter modeling (6-pulse rectifier)\n" +
                    "• FFT spectrum analysis of phase voltages and currents\n" +
                    "• Operating point computation: torque, power, efficiency\n" +
                    "• Phasor diagram visualization (dq reference frame)\n" +
                    "• Idealized vs. actual comparison with deviation highlighting"),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.Run(
                    "Enter motor parameters, inverter settings, and target dq voltages, " +
                    "then press 'Calculate!' to run the simulation. Hover over any label for a quick explanation of the parameter."),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.Run("📦 Libraries & Licenses")
                {
                    FontSize = 14,
                    FontWeight = Avalonia.Media.FontWeight.Bold,
                },
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.Run(
                    "• Avalonia UI 11.1.3 — Cross-platform .NET UI framework (MIT License)\n" +
                    "• ScottPlot 5.0.54 — Interactive charting library (MIT License)\n" +
                    "• CommunityToolkit.Mvvm 8.2.2 — MVVM source generators (MIT License)\n" +
                    "• Meta.Numerics 4.0.7 — Scientific numerics library (MIT License)\n" +
                    "• Fody 6.8.2 / Costura.Fody — IL weaving & assembly embedding (MIT License)"),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.Run(
                    "All libraries are open-source with permissive MIT licenses. " +
                    "The complete source code is available in the project repository."),
            }
        };

        var infoWindow = new Window
        {
            Title = "About PMSM Drive Calculator",
            Width = 520,
            Height = 480,
            MinWidth = 340,
            MinHeight = 320,
            WindowStartupLocation = WindowStartupLocation.CenterOwner,
            CanResize = true,
            Content = infoText,
        };
        infoWindow.ShowDialog(this);
    }

    private void OnHelpClick(object? sender, Avalonia.Interactivity.RoutedEventArgs e)
    {
        var helpWindow = new Window
        {
            Title = "Help – Theoretical Background & Tutorial",
            Width = 640,
            Height = 680,
            WindowStartupLocation = WindowStartupLocation.CenterOwner,
            Content = new ScrollViewer
            {
                HorizontalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Disabled,
                Content = new Border
                {
                    Padding = new Avalonia.Thickness(24),
                    Child = new StackPanel
                {
                    Spacing = 12,
                    Children =
                    {
                        new SelectableTextBlock { Text = "PMSM Drive Calculator — Help & Tutorial", FontSize = 20, FontWeight = Avalonia.Media.FontWeight.Bold, TextWrapping = Avalonia.Media.TextWrapping.Wrap },

                        Body("This application models star-connected (Y-connected) PMSM drives supplied by a three-phase symmetrical voltage-source PWM inverter circuit. All motor windings are Y-connected. When the LC output filter is enabled, the three filter capacitors are also Y-connected with a floating star point — identical to the motor topology. The system is 3-wire (no neutral return); all three line currents sum to zero at every node."),

                        // ═══════════════════════════════════════════
                        // SECTION 0: Quick Start Tutorial
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("🚀 Quick Start Tutorial"),

                        SubHeader("Step 1 — Enter Motor Parameters"),
                        Body("On the 'Motor' tab, set the pole pairs, PM flux linkage (ψPM in webers), d-axis and q-axis inductances (Ld, Lq in henries), phase resistance (Ω), Leakage Ratio (enabled only when LC output filter is active; used for high-frequency harmonic analysis through the filter), and speed (RPM). For surface-mount PMSM (SPM), set Ld ≈ Lq. For interior PMSM (IPM), Lq > Ld (saliency produces reluctance torque). Typical values: ψPM = 0.01–0.5 Wb, Ld/Lq = 0.1–100 mH, Rs = 0.01–10 Ω, Leakage Ratio = 0.05–0.50. The saturation ratio ξ = Lq/Ld is typically 1.0–1.2 for SPM and 1.5–4.0 for IPM."),

                        SubHeader("Step 2 — Configure the Inverter"),
                        Body("On the 'Inverter' tab, set the DC-link voltage (Vdc), switching frequency (Hz), the target voltage amplitude (V_LL_peak — this is the line-to-line peak = √3 × V_ph_peak), and the phase angle in degrees. Choose the PWM modulation strategy (SPWM2, IPSPWM3, SVPWM2, or SVPWM3). For extended voltage range, enable third-harmonic injection (only available with SPWM2 or IPSPWM3). The modulation index m = V_ph_peak / (Vdc/2) must stay ≤ 1.0 for SPWM2 and ≤ 1.155 for SVPWM2/IPSPWM3/SVPWM3.\n\n" +
                             "Note: SVPWM2 and SVPWM3 are internally implemented via QuasiSVPWM (zero-sequence injection / saddle waveform), which is mathematically equivalent to traditional space-vector PWM but avoids ZOH phase lag by evaluating the reference continuously at every sub-sample (200× per switching period)."),

                        SubHeader("Step 3 — (Optional) Configure Filters"),
                        Body("• LC Output Filter: Insert an LC low-pass filter between inverter and motor terminals. Set the filter inductance Lf (H) and filter capacitance Cf (F, Y-connected per-phase). Enable the checkbox — the solver auto-selects 'Full-Transient (DQ 6-State)'. Design tip: choose cutoff fc = 1/(2π√(Lf·Cf)) such that f_base ≪ fc ≪ fsw (typically fc ≈ fsw/5 to fsw/10). The Leakage Ratio on the Motor tab becomes enabled. The Leakage Ratio parameter sets the leakage inductance used for high-frequency harmonic analysis through the filter.\n" +
                             "• Grid-side DC-link Filter: Model the grid-side three-phase rectifier ripple on the DC link with Ldc (H), Cdc (F), grid voltage amplitude (Vp), and grid frequency (50/60 Hz). The ripple at 6× grid frequency modulates the PWM output. Typical Cdc = 1–10 mF, Ldc = 0.1–5 mH."),

                        SubHeader("Step 4 — Choose Solver & Calculate"),
                        Body("The solver type is auto-selected based on your filter configuration and shown (disabled) on the 'Solver' tab. Without LC filter: 'Transient (DQ)' — the DQ 2×2 backward Euler solver. With LC filter enabled: 'Full-Transient (DQ 6-State)' — the coupled 6-state DQ solver. Press the blue 'Calculate!' button — a progress bar will appear. The solver runs on a background thread, so the UI stays responsive."),

                        SubHeader("Step 5 — Interpret the Results"),
                        Body("• Operating Point Table (left panel after calculation): 'Idealized' column = steady-state dq solution. 'Actual' column = extracted from time-domain simulation. If |Idealized − Actual| > 0.5 V for any of Ud, Uq, or voltage magnitude, the 'Actual' values turn orange (#E67E22) to flag significant deviation.\n" +
                             "• Time-Domain Plots: PWM inverter voltages (line-to-line), motor terminal voltages (after LC filter if enabled), and motor phase currents vs. time. Look for steady-state convergence (no drift in amplitude/envelope).\n" +
                             "• FFT Plots: Harmonic spectra of voltages and currents, shown as frequency (Hz). The fundamental should dominate; switching harmonics appear at fsw ± sidebands. Check that filter attenuation is as expected.\n" +
                             "• Phasor Diagrams: Visualize dq voltage/current vectors. 'Actual' shows the real operating point; 'Idealized' shows the steady-state target. Compare angles and magnitudes between the two diagrams."),

                        // ═══════════════════════════════════════════
                        // SECTION 1: PMSM dq-Frame Model
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("📘 1. PMSM dq-Frame Model"),

                        Body("The motor is modeled in the rotor synchronous dq reference frame. The d-axis aligns with the rotor magnetic axis (PM flux direction); the q-axis leads by 90 electrical degrees."),

                        SubHeader("Steady-State Voltage Equations"),
                        Body("  Ud = Rs·Id − ω·Lq·Iq\n  Uq = Rs·Iq + ω·Ld·Id + ω·ψPM\n\n" +
                             "where:\n" +
                             "  Rs  = stator phase resistance (Ω)\n" +
                             "  Ld  = d-axis inductance (H)\n" +
                             "  Lq  = q-axis inductance (H)\n" +
                             "  ψPM = permanent-magnet flux linkage (Wb)\n" +
                             "  ω   = electrical angular speed = 2π·f_base = 2π·(RPM/60)·(poles/2)  (rad/s)"),

                        SubHeader("Torque Equation"),
                        Body("  Te = 1.5 · (poles/2) · [ ψPM·Iq + (Ld − Lq)·Id·Iq ]\n\n" +
                             "  Term 1: ψPM·Iq = magnet alignment torque (dominant term)\n" +
                             "  Term 2: (Ld−Lq)·Id·Iq = reluctance torque (non-zero only for salient-pole IPM machines)"),

                        SubHeader("Current Solution — Matrix Inversion"),
                        Body("  ┌ Rs   −ωLq ┐ ┌ Id ┐   ┌ Ud          ┐\n" +
                             "  └ ωLd   Rs  ┘ └ Iq ┘ = └ Uq − ω·ψPM ┘\n\n" +
                             "  det = Rs² + ω²·Ld·Lq\n" +
                             "  Id = (Rs·Ud + ωLq·(Uq − ωψPM)) / det\n" +
                             "  Iq = (Rs·(Uq − ωψPM) − ωLd·Ud) / det\n\n" +
                             "For ω = 0 (stall), the system degenerates to purely resistive: Id = Ud/Rs, Iq = Uq/Rs."),

                        // ═══════════════════════════════════════════
                        // SECTION 2: PWM Modulation
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("📘 2. PWM Modulation Strategies"),

                        Body("The inverter synthesizes the target dq voltage vector by generating pulse-width modulated (PWM) phase voltages. The DC-link voltage Vdc and switching frequency fsw determine the achievable output voltage resolution and harmonic spectrum."),

                        SubHeader("SPWM2 — Sinusoidal PWM (2-level)"),
                        Body("Each phase is compared against a sinusoidal reference. The output switches between +Vdc/2 and −Vdc/2, generating a bipolar pulse train. Maximum linear modulation: phase voltage amplitude = Vdc/2 (modulation index m = 1). Above m = 1, overmodulation occurs: the pulses merge near the sine peak, introducing low-order harmonics (5th, 7th, 11th...). Simple to implement but DC-link utilization is only 50%. The zero-sequence (common-mode) voltage is uncontrolled and contains substantial low-frequency content."),

                        SubHeader("IPSPWM3 — In-Phase SPWM (3-level)"),
                        Body("A 3-level carrier-based sinusoidal PWM with two in-phase triangular carriers: an upper carrier sweeping from 0 to +Vdc/2 and a lower carrier sweeping from −Vdc/2 to 0. Both carriers rise and fall in unison (in-phase), producing a three-level output: +Vdc/2 when the reference exceeds the upper carrier, −Vdc/2 when below the lower carrier, and 0 when between them. The 3-level output reduces dv/dt stress and improves harmonic quality compared to 2-level SPWM2. Optional 1/6 third-harmonic injection is available (same as SPWM2) to extend the linear modulation range by ~15.5%, achieving V_ph_peak = Vdc/√3 ≈ 0.577·Vdc."),

                        SubHeader("SVPWM2 — Space Vector PWM (2-level)"),
                        Body("Based on the concept of a rotating space vector in the αβ plane. The 8 switching states (000–111) form 6 sectors of a hexagon. The reference vector Vref is synthesized from the two adjacent active vectors and two zero vectors within each switching period Ts:\n\n" +
                             "  Vref·Ts = Vk·Tk + V(k+1)·T(k+1) + V0·T0/2 + V7·T7/2\n\n" +
                             "  Dwell times: Tk = Ts·m·sin(60°−θ'), T(k+1) = Ts·m·sin(θ')\n" +
                             "  Zero vector time: T0 = Ts − Tk − T(k+1), split equally between 000 and 111\n\n" +
                             "Uses a 7-segment symmetric switching pattern to minimize THD. Maximum linear modulation: V_ph_peak = Vdc/√3 (m_max ≈ 1.155). Overmodulation region I (1.155 < m < 1.5): zero vectors eliminated, partially distorted sine.\n\n" +
                             "Implementation note: SVPWM2 is implemented via QuasiSVPWM2 (zero-sequence injection / saddle waveform), which produces mathematically identical output voltages while avoiding zero-order-hold phase lag. The reference is evaluated continuously at every sub-sample (200× per switching period)."),

                        SubHeader("SVPWM3 — Space Vector PWM (3-level NPC)"),
                        Body("Extends SVPWM to a 3-level Neutral-Point-Clamped (NPC) inverter with 27 switching states (3³ = 27). The space vector diagram has 6 large triangular zones, each subdivided into 4 smaller regions (A, B, C, D). Switching states are labeled by per-phase level (−1: negative bus, 0: neutral point, +1: positive bus).\n\n" +
                             "Key advantages over 2-level:\n" +
                             "  • 3 voltage levels → 50% smaller voltage steps → ~75% lower dv/dt stress\n" +
                             "  • THD is typically 40–60% lower than equivalent 2-level\n" +
                             "  • Neutral point voltage balancing becomes an additional control objective\n" +
                             "  • Higher DC-link voltage possible with the same semiconductor rating\n\n" +
                             "Implementation note: SVPWM3 is implemented via QuasiSVPWM3 (zero-sequence injection / saddle waveform), which produces mathematically identical output voltages while avoiding zero-order-hold phase lag. The reference is evaluated continuously at every sub-sample (200× per switching period)."),

                       SubHeader("Why QuasiSVPWM? — Zero-Order-Hold Phase Lag"),
                       Body("Traditional SVPWM (subdomain/sector-based computation) samples the reference voltage vector once per switching period (at period-start, t = i/fsw) and holds that value for all 200 sub-samples within the period. This zero-order-hold (ZOH) introduces an effective phase lag of δθ = ω·Ts/2 (half the switching period). At f_base = 200 Hz, fsw = 8 kHz: δθ = 4.5°.\n\n" +
                            "  Ts = 1/fsw = 125 μs,  ω = 2π·200 = 1256.6 rad/s\n" +
                            "  δθ = ω·Ts/2 = 1256.6 × 62.5×10⁻⁶ = 0.0785 rad = 4.5°\n\n" +
                            "This phase lag means the average PWM output voltage over the switching period corresponds to the reference at t + Ts/2, not t. The solver's sync-demodulated current then disagrees with the steady-state prediction by approximately 10% at typical parameters (32.6 A vs. 35.9 A at default settings).\n\n" +
                            "QuasiSVPWM (zero-sequence injection / saddle waveform) is mathematically equivalent to traditional SVPWM but fundamentally avoids the ZOH issue:\n\n" +
                            "  • Formula:  u_zero = (max(ua,ub,uc) + min(ua,ub,uc)) / 2\n" +
                            "             ua_new = ua − u_zero  (same for phases b, c)\n\n" +
                            "  • The zero-sequence signal u_zero contains 3rd, 9th, 15th, … harmonics that cancel in the line-to-line voltage, so the motor sees identical fundamental and switching harmonics as traditional SVPWM.\n" +
                            "  • Because QuasiSVPWM computes the reference from continuous sinusoidal functions (not a sampled-and-held space vector), the reference is evaluated at every 625 ns sub-sample — continuously, just like SPWM2 and IPSPWM3.\n" +
                            "  • This eliminates the half-period ZOH lag entirely. The solver's sync-demodulated current now matches the steady-state prediction within numerical precision.\n\n" +
                            "In the GUI, selecting 'SVPWM2' or 'SVPWM3' automatically dispatches to QuasiSVPWM2 or QuasiSVPWM3 internally. The original SVPWM2/SVPWM3 classes (with subdomain/sector computation) are preserved in the source code for reference but are not used by the GUI."),

                      // ═══════════════════════════════════════════
                      // SECTION 3: LC Output Filter
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("📘 3. LC Output Filter"),

                        Body("A Y-connected LC low-pass filter can be placed between the inverter output and motor terminals. The three filter capacitors are Y-connected with a floating star point — identical to the motor winding topology. This 3-wire (no neutral) configuration is assumed throughout all models in this application. It attenuates PWM switching-frequency harmonics, reducing motor insulation stress, bearing currents, and high-frequency iron losses."),

                        SubHeader("Circuit Topology"),
                        Body("  Inverter Phase U → [Lf] → Motor Terminal U\n" +
                             "                          → [Cf] → Capacitor Star Point (floating)\n\n" +
                             "  The system is 3-wire (no neutral return). All three line currents sum to zero at every node:\n" +
                             "    i_Lf_u + i_Lf_v + i_Lf_w = 0\n" +
                             "    i_m_u  + i_m_v  + i_m_w  = 0\n" +
                             "    v_Cf_u + v_Cf_v + v_Cf_w = 0\n\n" +
                             "  Because of these constraints, only U and V phases are independent; W is derived."),

                        SubHeader("Frequency-Domain Transfer Function"),
                        Body("The transfer function H(jω) describes the LC filter in the frequency domain:\n" +
                             "  H(jω) = 1 / (1 − ω²·Lf·Cf + jω·Lf/Z_motor)\n\n" +
                             "  Attenuation at switching frequency fsw:\n" +
                             "    |H(j·2π·fsw)| ≈ 1 / (ωsw²·Lf·Cf)  for ωsw²·Lf·Cf ≫ 1\n\n" +
                             "  The filter introduces a voltage drop and phase shift at the fundamental frequency. The 'Idealized' column accounts for this by solving the steady-state dq equations with the filter-corrected motor terminal voltages."),

                        SubHeader("Full-Transient LC Filter Solver (DQ 6-State)"),
                        Body("The 'Full-Transient (DQ 6-State)' solver integrates the LC filter and motor as a single coupled 6-state system directly in the dq reference frame:\n\n" +
                             "  State variables: i_Lf_d, i_Lf_q, v_Cf_d, v_Cf_q, i_m_d, i_m_q\n\n" +
                             "  Algorithm (sequential semi-implicit per time step):\n" +
                             "    Step A — Motor 2×2 backward Euler solve with v_Cf[k-1] (predictor)\n" +
                             "    Step B — Capacitor update (explicit Euler for cross terms)\n" +
                             "    Step C — Inductor update (explicit Euler for cross terms)\n" +
                             "    Step D — Corrector: re-solve motor 2×2 with updated v_Cf[k]\n\n" +
                             "  The ABC→DQ transform uses a zero-sequence-rejecting Clarke transform:\n" +
                             "    v_α = (2·V_inv_u − V_inv_v − V_inv_w) / 3\n" +
                             "    v_β = (V_inv_v − V_inv_w) / √3\n\n" +
                             "  This prevents 3rd harmonic common-mode voltage from leaking into the dq frame.\n" +
                             "  Per-step cost: ~30 multiplications, ~20 additions — O(1) per time step.\n\n" +
                             "  DQ→ABC output conversion uses inverse Park+Clarke to produce 3-phase motor currents, capacitor voltages, and filter inductor currents."),

                        // ═══════════════════════════════════════════
                        // SECTION 4: Grid-Side DC-Link Ripple Filter
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("📘 4. Grid-Side DC-Link Ripple Filter"),

                        Body("Models a three-phase diode bridge rectifier feeding an LC DC-link filter. The rectifier produces a characteristic 6-pulse ripple at 6× the grid frequency (300 Hz for 50 Hz grid, 360 Hz for 60 Hz).\n\n" +
                             "Parameters:\n" +
                             "  • Grid voltage amplitude Vp = Vrms × √2 (e.g., 230Vrms → 325V)\n" +
                             "  • Grid frequency (50 or 60 Hz)\n" +
                             "  • DC-link capacitance Cdc (F) — typically mF range\n" +
                             "  • DC-link inductance Ldc (H) — typically μH to mH range\n\n" +
                             "The load current Idc is estimated from the motor active power:\n" +
                             "  Idc ≈ P_motor / Vdc\n\n" +
                             "The ripple voltage ΔVdc modulates the PWM output amplitude, creating low-frequency sidebands around the switching harmonics in the FFT spectrum."),

                        // ═══════════════════════════════════════════
                        // SECTION 5: Solvers
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("📘 5. Solver Types"),

                        Body("All solvers operate in the dq (synchronous) reference frame. The Clarke transform used for ABC→DQ conversion is zero-sequence-rejecting: v_α = (2U−V−W)/3, v_β = (V−W)/√3. This ensures that 3rd harmonic common-mode voltage (from SPWM third-harmonic injection) is canceled and does not produce non-physical 3rd harmonic currents."),

                        SubHeader("Transient (DQ) Solver"),
                        Body("DQ-frame 2×2 backward Euler time-domain simulation. The motor dq voltage equations:\n\n" +
                             "  Ld·di_d/dt = v_d − R·i_d + ω·Lq·i_q\n" +
                             "  Lq·di_q/dt = v_q − R·i_q − ω·Ld·i_d − ω·ψPM\n\n" +
                             "are discretized via backward Euler, producing a 2×2 linear system per time step:\n\n" +
                             "  ┌ 1+dt·R/Ld   −dt·ω·Lq/Ld ┐ ┌ i_d[k] ┐   ┌ i_d[k−1] + dt·v_d/Ld          ┐\n" +
                             "  └ dt·ω·Ld/Lq   1+dt·R/Lq  ┘ └ i_q[k] ┘ = └ i_q[k−1] + dt·(v_q−ω·ψPM)/Lq ┘\n\n" +
                             "Solved via Cramer's rule (det ≈ 1 + dt·R·(1/Ld+1/Lq) for small dt). The ABC→DQ voltage conversion uses a zero-sequence-rejecting Clarke+Park transform. Currents are converted back to ABC via inverse Park+Clarke.\n\n" +
                             "Per-step cost: ~12 multiplications, ~8 additions, 1 division — O(1). Uses trigonometric recurrence (sin/cos rotation) for angle propagation, avoiding expensive Math.Sin/Cos calls."),

                        SubHeader("Full-Transient LC Filter Solver (DQ 6-State)"),
                        Body("Integrates the LC filter and motor as a single coupled 6-state system directly in the dq reference frame. Algorithm per time step:\n\n" +
                             "  Step A — Motor 2×2 backward Euler solve with v_Cf[k−1] (predictor)\n" +
                             "  Step B — Capacitor update (explicit Euler for cross terms, implicit for through terms)\n" +
                             "  Step C — Inductor update (explicit Euler for cross terms, implicit for through terms)\n" +
                             "  Step D — Corrector: re-solve motor 2×2 with updated v_Cf[k]\n\n" +
                             "  All ABC→DQ conversions use the zero-sequence-rejecting Clarke transform.\n" +
                             "  Per-step cost: ~30 multiplications, ~20 additions — O(1) per time step.\n\n" +
                             "  Solver selection guide:\n" +
                             "    • Transient (DQ): Default — accurate for systems without LC filter\n" +
                             "    • Full-Transient (DQ 6-State): With LC output filter enabled — captures true coupled dynamics"),

                        // ═══════════════════════════════════════════
                        // SECTION 6: Phasor Diagrams
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("📘 6. Phasor Diagrams (dq Frame)"),

                        Body("Two phasor plots are shown on the 'Phasor Diagrams' tab:\n" +
                             "  • 'Actual' — Built from the time-domain extracted dq voltages and currents.\n" +
                             "  • 'Idealized' — Built from the steady-state solution using the target (or LC-filter-corrected) dq voltages.\n\n" +
                             "Each phasor diagram displays:\n" +
                             "  Currents (blue arrows):\n" +
                             "    • Id — d-axis current (on d-axis)\n" +
                             "    • Iq — q-axis current (on q-axis, 90° from d)\n" +
                             "    • Im — Resultant current magnitude (thick blue arrow)\n\n" +
                             "  Voltage chain (colored arrows, head-to-tail):\n" +
                             "    • jω·ψPM — Back-EMF (on q-axis, orange)\n" +
                             "    • jωLq·Iq — q-axis inductive drop (purple)\n" +
                             "    • jωLd·Id — d-axis inductive drop (teal)\n" +
                             "    • R·Id — d-axis resistive drop (red)\n" +
                             "    • R·Iq — q-axis resistive drop (green)\n" +
                             "    • Um — Resultant motor terminal voltage (thick red arrow)\n\n" +
                             "  Crosshair lines mark the d-axis (horizontal) and q-axis (vertical). Projection lines from the voltage tip to the axes show Ud and Uq components.\n\n" +
                             "Comparing the two diagrams reveals the effect of PWM distortion, filter voltage drop, and solver accuracy on the operating point."),

                        // ═══════════════════════════════════════════
                        // SECTION 7: Idealized vs. Actual
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("📘 7. Idealized vs. Actual Comparison"),

                        Body("The operating point table shows side-by-side columns:\n\n" +
                             "  Idealized — Computed from the steady-state dq model using the target Ud/Uq (with LC filter correction if enabled). This represents what the drive should theoretically achieve with perfect sinusoidal PWM output.\n\n" +
                             "  Actual — Extracted from the time-domain simulation results. Includes PWM distortion, filter dynamics, solver discretization, and transient effects.\n\n" +
                             "Deviation highlighting: If |Idealized − Actual| > 0.5 V for any of Ud, Uq, or Voltage Magnitude, the 'Actual' column values turn orange (#E67E22) to alert the user. This typically occurs when:\n" +
                             "  • Modulation index is too high (overmodulation — voltage clipping)\n" +
                             "  • Switching frequency is too low relative to fundamental (insufficient samples per period)\n" +
                             "  • LC filter causes a significant fundamental voltage drop\n" +
                             "  • Solver hasn't reached steady state (increase number of periods)\n" +
                             "  • DC-link ripple is modulating the PWM output"),

                        // ═══════════════════════════════════════════
                        // SECTION 8: FFT Spectral Analysis
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("📘 8. FFT Spectral Analysis"),

                        Body("Fast Fourier Transform (FFT) is computed on the last few periods of steady-state waveforms using a Hanning window. The spectrum is displayed as frequency (Hz) vs. magnitude.\n\n" +
                             "Three FFT plot groups:\n" +
                             "  1. PWM Output Voltage FFT — Shows switching frequency harmonics and sidebands. The fundamental component (f_base) should match the target modulation amplitude.\n" +
                             "  2. Motor Terminal Voltage FFT — After the LC filter (if enabled). Switching harmonics are attenuated by the filter roll-off. Compare with PWM FFT to quantify filter effectiveness.\n" +
                             "  3. Motor Current FFT — Currents are naturally low-pass filtered by the motor inductance (Leq). The spectrum should be dominated by the fundamental with minimal harmonics.\n\n" +
                             "The X-axis in FFT plots extends to 4× the switching frequency to show the full harmonic spectrum. The fundamental component is subtracted before display to isolate harmonic distortion content.\n\n" +
                             "Windowing: A Hanning window w[n] = 0.5·(1 − cos(2πn/N)) is applied to reduce spectral leakage from non-integer-period sampling. The window is applied to the last N samples (user-configurable periods × samples per period)."),

                        // ═══════════════════════════════════════════
                        // SECTION 9: Operating Point Calculations
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("📘 9. Operating Point Calculations"),

                        Body("The 'Operating Point' table displays key performance metrics computed from both idealized and actual dq quantities:\n\n" +
                             "  • Torque (Nm) — Te from the torque equation (Section 1)\n" +
                             "  • Mechanical Power (W) — Pmech = Te × ω_mech = Te × (RPM × 2π/60)\n" +
                             "  • Electrical Power (W) — Pel = 1.5 × (Ud·Id + Uq·Iq) (dq power invariant scaling)\n" +
                             "  • Efficiency (%) — η = Pmech / Pel × 100. Accounts for copper losses (1.5·Rs·(Id²+Iq²)).\n" +
                             "  • Power Factor — cos(φ) = Pel / (3/2 × Vrms × Irms) = (Ud·Id+Uq·Iq) / (|V_dq|·|I_dq|)\n" +
                             "  • Current Magnitude — |I_dq| = √(Id² + Iq²)\n" +
                             "  • Voltage Magnitude — |V_dq| = √(Ud² + Uq²)\n\n" +
                             "The 'Actual' values are derived from the time-domain steady-state extraction (averaged over the last few simulation periods). The 'Idealized' values come directly from the dq steady-state solver.\n\n" +
                             "Copper losses: Pcu = 1.5 × Rs × (Id² + Iq²). Iron losses (eddy current + hysteresis) are not explicitly modeled in this version."),

                        // ═══════════════════════════════════════════
                        // SECTION 10: Verification & Validation
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("📘 10. Verification & Validation Tips"),

                        Body("To verify that simulation results are physically meaningful, check the following:\n\n" +
                             "  1. DC case (ω = 0): Set speed to 0 RPM. The motor becomes purely resistive — Id = Ud/Rs, Iq = Uq/Rs. Torque = 1.5×(p/2)×ψPM×Iq (no reluctance term). Verify this analytically.\n\n" +
                             "  2. No-load (Id = Iq ≈ 0): Set Ud = 0, Uq = ω·ψPM. The motor draws minimal current — only to overcome friction/windage (not modeled). Torque should be near zero.\n\n" +
                             "  3. SPM vs IPM: For Ld ≈ Lq, set both to the same value. The reluctance torque term should vanish. For Lq > Ld with negative Id (field weakening), check that total torque > magnet torque alone.\n\n" +
                             "  4. Filter effect: With LC filter enabled, motor terminal voltages should show reduced harmonic content vs. inverter output. The fundamental amplitude should drop slightly due to the filter voltage divider.\n\n" +
                             "  5. Solver convergence: Increase the number of periods and verify that Actual values approach Idealized values (should converge within 0.5 V for well-designed systems).\n\n" +
                             "  6. Modulation limit: Set target voltage above the linear modulation limit. Observe overmodulation effects: voltage clipping in time-domain plots, increased low-order harmonics in FFT, and orange-highlighted deviation in the operating point table."),

                        // ═══════════════════════════════════════════
                        // SECTION 11: Troubleshooting
                        // ═══════════════════════════════════════════
                        Sep(), SectionHeader("📘 11. Troubleshooting Common Issues"),

                        SubHeader("Orange-highlighted values (deviation > 0.5 V)"),
                        Body("• Reduce target voltage or increase DC-link voltage to stay within linear modulation range.\n• Increase switching frequency (more samples per fundamental period for better accuracy).\n• Increase the number of simulated periods for better steady-state convergence.\n• If using LC filter, check that the filter cutoff is well above fundamental but well below switching frequency."),

                        SubHeader("Very low efficiency"),
                        Body("• Check phase resistance Rs — high resistance causes large copper losses proportional to I².\n• Verify that the motor is not operating in deep field weakening (large negative Id increases current magnitude without producing useful torque).\n• The operating point may be far from the optimal MTPA (Maximum Torque Per Ampere) trajectory."),

                        SubHeader("FFT shows unexpected harmonics"),
                        Body("• Overmodulation: fundamental > linear limit → low-order harmonics (5th, 7th, 11th, 13th).\n• Insufficient simulation time: transients not decayed. Increase number of periods.\n• Grid-side ripple: check for 300/360 Hz sidebands around switching harmonics.\n• Aliasing: ensure samples per period × f_base ≪ fsw (at least 20× margin)."),

                        SubHeader("Phasor diagrams look wrong"),
                        Body("• Current phasor angle should be approximately 90° lagging voltage in inductive region.\n• Back-EMF arrow (jω·ψPM) should always point along q-axis with magnitude proportional to speed.\n• If motor terminal voltage is much smaller than back-EMF, the motor may be generating (regenerative braking).\n• For IPM with negative Id, the voltage phasor map shifts due to cross-coupling terms."),

                        SubHeader("Calculation seems stuck or too slow"),
                        Body("• Both DQ solvers are O(N) per time step — total time ∝ (periods × samples_per_period). Reduce either.\n• The DQ 6-state LC filter solver is ~2.5× slower than the DQ 2×2 solver (6 states vs. 2 states).\n• The calculation runs on a background thread — the UI remains responsive during computation."),

                    }
                }
            }
        }
        };
        helpWindow.ShowDialog(this);
    }

    // ── Help window helper methods ──
    private static SelectableTextBlock SectionHeader(string text)
    {
        return new SelectableTextBlock
        {
            Text = text,
            FontSize = 15,
            FontWeight = Avalonia.Media.FontWeight.Bold,
            Foreground = new Avalonia.Media.SolidColorBrush(Avalonia.Media.Color.FromRgb(37, 99, 235)),
            TextWrapping = Avalonia.Media.TextWrapping.Wrap
        };
    }

    private static SelectableTextBlock SubHeader(string text)
    {
        return new SelectableTextBlock
        {
            Text = text,
            FontWeight = Avalonia.Media.FontWeight.Bold,
            FontSize = 13,
            Margin = new Avalonia.Thickness(0, 6, 0, 0),
            TextWrapping = Avalonia.Media.TextWrapping.Wrap
        };
    }

    private static SelectableTextBlock Body(string text)
    {
        return new SelectableTextBlock
        {
            Text = text,
            FontSize = 12,
            TextWrapping = Avalonia.Media.TextWrapping.Wrap,
            Foreground = new Avalonia.Media.SolidColorBrush(Avalonia.Media.Color.FromRgb(31, 41, 55))
        };
    }

    private static Separator Sep()
    {
        return new Separator
        {
            Height = 1,
            Background = new Avalonia.Media.SolidColorBrush(Avalonia.Media.Color.FromRgb(229, 231, 235)),
            Margin = new Avalonia.Thickness(0, 4)
        };
    }

    private void InitializeAllPlots()
    {
        InitPhasorPlot();
        InitPhasorPlotIdealized();
        InitPwmVoltageTimePlot();
        InitPwmVoltageFFTPlot();
        InitMotorVoltageTimePlot();
        InitMotorVoltageFFTPlot();
        InitCurrentTimePlot();
        InitCurrentFFTPlot();
    }

    // Keep references to crosshair lines so we can preserve them on refresh
    // -- Actual phasor plot
    private IPlottable? _phasorHAxis;
    private IPlottable? _phasorVAxis;
    private double _phasorPlotLim = 250;
    // -- Idealized phasor plot
    private IPlottable? _phasorHAxisIdealized;
    private IPlottable? _phasorVAxisIdealized;
    private double _phasorPlotLimIdealized = 250;

    // ═══════════════════════════════════════════════════════
    // Phasor Diagram (dq frame)
    // ═══════════════════════════════════════════════════════
    private void InitPhasorPlot()
    {
        var plt = PhasorPlot.Plot;
        ApplyWhiteStyle(plt);
        plt.XLabel("d");
        plt.YLabel("q");
        plt.Axes.SetLimits(-_phasorPlotLim, _phasorPlotLim, -_phasorPlotLim, _phasorPlotLim);
        plt.Axes.SquareUnits();

        DrawPhasorCrosshairs();
        PhasorPlot.Refresh();
    }

    private void DrawPhasorCrosshairs()
    {
        var plt = PhasorPlot.Plot;
        double L = _phasorPlotLim;

        // Remove old crosshairs if any
        if (_phasorHAxis != null) plt.Remove(_phasorHAxis);
        if (_phasorVAxis != null) plt.Remove(_phasorVAxis);

        // Horizontal axis line at y=0 (using Scatter with 2 points)
        _phasorHAxis = plt.Add.Scatter(new double[] { -L, L }, new double[] { 0, 0 }, Colors.Gray);
        if (_phasorHAxis is ScottPlot.Plottables.Scatter sh)
        {
            sh.LinePattern = LinePattern.Dotted;
            sh.LineWidth = 1;
            sh.MarkerSize = 0;
        }

        // Vertical axis line at x=0 (using Scatter with 2 points)
        _phasorVAxis = plt.Add.Scatter(new double[] { 0, 0 }, new double[] { -L, L }, Colors.Gray);
        if (_phasorVAxis is ScottPlot.Plottables.Scatter sv)
        {
            sv.LinePattern = LinePattern.Dotted;
            sv.LineWidth = 1;
            sv.MarkerSize = 0;
        }
    }

    private void DrawPhasorCrosshairsIdealized()
    {
        var plt = PhasorPlotIdealized.Plot;
        double L = _phasorPlotLimIdealized;

        // Remove old crosshairs if any
        if (_phasorHAxisIdealized != null) plt.Remove(_phasorHAxisIdealized);
        if (_phasorVAxisIdealized != null) plt.Remove(_phasorVAxisIdealized);

        // Horizontal axis line at y=0
        _phasorHAxisIdealized = plt.Add.Scatter(new double[] { -L, L }, new double[] { 0, 0 }, Colors.Gray);
        if (_phasorHAxisIdealized is ScottPlot.Plottables.Scatter sh)
        {
            sh.LinePattern = LinePattern.Dotted;
            sh.LineWidth = 1;
            sh.MarkerSize = 0;
        }

        // Vertical axis line at x=0
        _phasorVAxisIdealized = plt.Add.Scatter(new double[] { 0, 0 }, new double[] { -L, L }, Colors.Gray);
        if (_phasorVAxisIdealized is ScottPlot.Plottables.Scatter sv)
        {
            sv.LinePattern = LinePattern.Dotted;
            sv.LineWidth = 1;
            sv.MarkerSize = 0;
        }
    }

    private void InitPhasorPlotIdealized()
    {
        var plt = PhasorPlotIdealized.Plot;
        ApplyWhiteStyle(plt);
        plt.XLabel("d");
        plt.YLabel("q");
        plt.Axes.SetLimits(-_phasorPlotLimIdealized, _phasorPlotLimIdealized, -_phasorPlotLimIdealized, _phasorPlotLimIdealized);
        plt.Axes.SquareUnits();

        DrawPhasorCrosshairsIdealized();
        PhasorPlotIdealized.Refresh();
    }

    private void RefreshPhasorPlot()
    {
        var plt = PhasorPlot.Plot;
        // Remove all plottables except crosshair references
        var plottables = plt.GetPlottables().ToList();
        foreach (var p in plottables)
        {
            if (p == _phasorHAxis || p == _phasorVAxis) continue;
            plt.Remove(p);
        }

        // Re-draw crosshairs if they don't exist (e.g. this is the first refresh)
        if (_phasorHAxis == null || _phasorVAxis == null)
        {
            DrawPhasorCrosshairs();
        }

        if (_vm.PhasorItems.Count == 0)
        {
            PhasorPlot.Refresh();
            return;
        }

        // ── Parse phasor items ──
        // Items 0-1: Id, Iq (currents, drawn from origin, thinner line)
        // Items 2-6: Voltage chain (drawn head-to-tail)
        //   Chain: jωψPM → jωLq·Iq → jωLd·Id → R·Id → R·Iq
        //   Tip lands at (Ud, Uq)

        // Current phasor color
        var currentColor = Color.FromHex("#3498DB");
        var chainColors = new[] { "#F39C12", "#9B59B6", "#1ABC9C", "#E74C3C", "#2ECC71" };

        var idItem = _vm.PhasorItems[0];
        var iqItem = _vm.PhasorItems[1];

        double idX = idItem.Magnitude * Math.Cos(idItem.AngleRad);
        double idY = idItem.Magnitude * Math.Sin(idItem.AngleRad);
        double iqDx = iqItem.Magnitude * Math.Cos(iqItem.AngleRad);
        double iqDy = iqItem.Magnitude * Math.Sin(iqItem.AngleRad);
        double imX = idX + iqDx;
        double imY = idY + iqDy;

        // ── Auto-scale: use the overall max magnitude from all items ──
        double maxMag = _vm.PhasorItems.Max(p => p.Magnitude);
        double lim = Math.Max(maxMag * 1.4, 10);
        plt.Axes.Bottom.Min = -lim;
        plt.Axes.Bottom.Max = lim;
        plt.Axes.Left.Min = -lim;
        plt.Axes.Left.Max = lim;
        plt.Axes.SquareUnits();

        // ── Axis labels: "d" and "q" ──
        var labelD = plt.Add.Text("d", lim * 0.95, lim * -0.08);
        labelD.LabelFontSize = 13;
        labelD.LabelFontColor = Colors.DimGray;
        labelD.LabelItalic = true;
        var labelQ = plt.Add.Text("q", lim * -0.08, lim * 0.92);
        labelQ.LabelFontSize = 13;
        labelQ.LabelFontColor = Colors.DimGray;
        labelQ.LabelItalic = true;

        // ── Draw current phasors head-to-tail: Id from origin → Iq from Id tip, single-line arrowhead ──
        // Id: origin → Id tip (thin line, small arrowhead)
        var idArrow = plt.Add.Arrow(new Coordinates(0, 0), new Coordinates(idX, idY));
        idArrow.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        idArrow.ArrowLineColor = currentColor;
        idArrow.ArrowLineWidth = 1.5f;
        idArrow.ArrowheadLength = 6;
        idArrow.ArrowheadWidth = 5;

        // Iq: Id tip → Id+Iq tip (same color, thin line)
        var iqArrow = plt.Add.Arrow(new Coordinates(idX, idY), new Coordinates(imX, imY));
        iqArrow.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        iqArrow.ArrowLineColor = currentColor;
        iqArrow.ArrowLineWidth = 1.5f;
        iqArrow.ArrowheadLength = 6;
        iqArrow.ArrowheadWidth = 5;

        // Resultant Im: origin → final tip (thicker line, larger arrowhead)
        var imArrow = plt.Add.Arrow(new Coordinates(0, 0), new Coordinates(imX, imY));
        imArrow.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        imArrow.ArrowLineColor = currentColor;
        imArrow.ArrowLineWidth = 2;
        imArrow.ArrowheadLength = 10;
        imArrow.ArrowheadWidth = 7;

        // Labels: Id at midpoint of its segment, Iq at midpoint of its segment
        var idLabel = plt.Add.Text(idItem.Name, idX * 0.5, idY * 0.55);
        idLabel.LabelFontSize = 10;
        idLabel.LabelFontColor = currentColor;

        var iqLabel = plt.Add.Text(iqItem.Name, (idX + imX) / 2 * 1.08, (idY + imY) / 2 * 1.08);
        iqLabel.LabelFontSize = 10;
        iqLabel.LabelFontColor = currentColor;

        // "Im" label near the resultant tip (bold, 12pt)
        var imLabel = plt.Add.Text("Im", imX * 1.08, imY * 1.08);
        imLabel.LabelFontSize = 12;
        imLabel.LabelFontColor = currentColor;
        imLabel.LabelBold = true;

        // ── Draw voltage chain head-to-tail, single-line arrowhead ──
        double tipX = 0, tipY = 0;
        for (int i = 0; i < 5; i++)
        {
            var item = _vm.PhasorItems[2 + i];
            double dx = item.Magnitude * Math.Cos(item.AngleRad);
            double dy = item.Magnitude * Math.Sin(item.AngleRad);
            double endX = tipX + dx;
            double endY = tipY + dy;
            var color = Color.FromHex(chainColors[i]);

            var arrow = plt.Add.Arrow(new Coordinates(tipX, tipY),
                new Coordinates(endX, endY));
            arrow.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
            arrow.ArrowLineColor = color;
            arrow.ArrowLineWidth = 2;
            arrow.ArrowheadLength = 8;
            arrow.ArrowheadWidth = 6;

            // Label at midpoint of segment
            var txt = plt.Add.Text(item.Name, (tipX + endX) / 2 * 1.05,
                (tipY + endY) / 2 * 1.05);
            txt.LabelFontSize = 10;
            txt.LabelFontColor = color;

            tipX = endX;
            tipY = endY;
        }

        // ── Draw resultant U = (Ud, Uq) from origin to final tip ──
        var uColor = Color.FromHex("#E74C3C");
        var uArrow = plt.Add.Arrow(new Coordinates(0, 0), new Coordinates(tipX, tipY));
        uArrow.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        uArrow.ArrowLineColor = uColor;
        uArrow.ArrowLineWidth = 2;
        uArrow.ArrowheadLength = 10;
        uArrow.ArrowheadWidth = 7;
        // "U" label near the tip
        var uLabel = plt.Add.Text("Um", tipX * 1.08, tipY * 1.08);
        uLabel.LabelFontSize = 12;
        uLabel.LabelFontColor = uColor;
        uLabel.LabelBold = true;

        // ── Draw Ud/Uq projections onto axes, single-line arrowhead ──
        var projUdColor = Colors.DimGray;
        var projUd = plt.Add.Arrow(new Coordinates(0, 0), new Coordinates(tipX, 0));
        projUd.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        projUd.ArrowLineColor = projUdColor;
        projUd.ArrowLineWidth = 1;
        projUd.ArrowheadLength = 5;
        projUd.ArrowheadWidth = 4;
        var projUdLabel = plt.Add.Text("Ud", tipX * 0.6, 0.06 * lim);
        projUdLabel.LabelFontSize = 9;
        projUdLabel.LabelFontColor = projUdColor;

        var projUq = plt.Add.Arrow(new Coordinates(0, 0), new Coordinates(0, tipY));
        projUq.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        projUq.ArrowLineColor = projUdColor;
        projUq.ArrowLineWidth = 1;
        projUq.ArrowheadLength = 5;
        projUq.ArrowheadWidth = 4;
        var projUqLabel = plt.Add.Text("Uq", 0.05 * lim, tipY * 0.65);
        projUqLabel.LabelFontSize = 9;
        projUqLabel.LabelFontColor = projUdColor;

        // ── Draw dotted construction lines from tip to axes ──
        var hLine = plt.Add.Scatter(new double[] { 0, tipX }, new double[] { tipY, tipY }, projUdColor);
        if (hLine is ScottPlot.Plottables.Scatter sh2)
        {
            sh2.LinePattern = LinePattern.Dotted;
            sh2.LineWidth = 0.8f;
            sh2.MarkerSize = 0;
        }
        var vLine = plt.Add.Scatter(new double[] { tipX, tipX }, new double[] { 0, tipY }, projUdColor);
        if (vLine is ScottPlot.Plottables.Scatter sv2)
        {
            sv2.LinePattern = LinePattern.Dotted;
            sv2.LineWidth = 0.8f;
            sv2.MarkerSize = 0;
        }

        PhasorPlot.Refresh();
    }

    private void RefreshPhasorPlotIdealized()
    {
        var plt = PhasorPlotIdealized.Plot;
        // Remove all plottables except crosshair references
        var plottables = plt.GetPlottables().ToList();
        foreach (var p in plottables)
        {
            if (p == _phasorHAxisIdealized || p == _phasorVAxisIdealized) continue;
            plt.Remove(p);
        }

        // Re-draw crosshairs if they don't exist
        if (_phasorHAxisIdealized == null || _phasorVAxisIdealized == null)
        {
            DrawPhasorCrosshairsIdealized();
        }

        if (_vm.PhasorItemsIdealized.Count == 0)
        {
            PhasorPlotIdealized.Refresh();
            return;
        }

        // ── Parse phasor items ──
        var currentColor = Color.FromHex("#3498DB");
        var chainColors = new[] { "#F39C12", "#9B59B6", "#1ABC9C", "#E74C3C", "#2ECC71" };

        var idItem = _vm.PhasorItemsIdealized[0];
        var iqItem = _vm.PhasorItemsIdealized[1];

        double idX = idItem.Magnitude * Math.Cos(idItem.AngleRad);
        double idY = idItem.Magnitude * Math.Sin(idItem.AngleRad);
        double iqDx = iqItem.Magnitude * Math.Cos(iqItem.AngleRad);
        double iqDy = iqItem.Magnitude * Math.Sin(iqItem.AngleRad);
        double imX = idX + iqDx;
        double imY = idY + iqDy;

        // ── Auto-scale ──
        double maxMag = _vm.PhasorItemsIdealized.Max(p => p.Magnitude);
        double lim = Math.Max(maxMag * 1.4, 10);
        plt.Axes.Bottom.Min = -lim;
        plt.Axes.Bottom.Max = lim;
        plt.Axes.Left.Min = -lim;
        plt.Axes.Left.Max = lim;
        plt.Axes.SquareUnits();

        // ── Axis labels ──
        var labelD = plt.Add.Text("d", lim * 0.95, lim * -0.08);
        labelD.LabelFontSize = 13;
        labelD.LabelFontColor = Colors.DimGray;
        labelD.LabelItalic = true;
        var labelQ = plt.Add.Text("q", lim * -0.08, lim * 0.92);
        labelQ.LabelFontSize = 13;
        labelQ.LabelFontColor = Colors.DimGray;
        labelQ.LabelItalic = true;

        // ── Current phasors head-to-tail ──
        var idArrow = plt.Add.Arrow(new Coordinates(0, 0), new Coordinates(idX, idY));
        idArrow.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        idArrow.ArrowLineColor = currentColor;
        idArrow.ArrowLineWidth = 1.5f;
        idArrow.ArrowheadLength = 6;
        idArrow.ArrowheadWidth = 5;

        var iqArrow = plt.Add.Arrow(new Coordinates(idX, idY), new Coordinates(imX, imY));
        iqArrow.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        iqArrow.ArrowLineColor = currentColor;
        iqArrow.ArrowLineWidth = 1.5f;
        iqArrow.ArrowheadLength = 6;
        iqArrow.ArrowheadWidth = 5;

        var imArrow = plt.Add.Arrow(new Coordinates(0, 0), new Coordinates(imX, imY));
        imArrow.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        imArrow.ArrowLineColor = currentColor;
        imArrow.ArrowLineWidth = 2;
        imArrow.ArrowheadLength = 10;
        imArrow.ArrowheadWidth = 7;

        var idLabel = plt.Add.Text(idItem.Name, idX * 0.5, idY * 0.55);
        idLabel.LabelFontSize = 10;
        idLabel.LabelFontColor = currentColor;

        var iqLabel = plt.Add.Text(iqItem.Name, (idX + imX) / 2 * 1.08, (idY + imY) / 2 * 1.08);
        iqLabel.LabelFontSize = 10;
        iqLabel.LabelFontColor = currentColor;

        var imLabel = plt.Add.Text("Im", imX * 1.08, imY * 1.08);
        imLabel.LabelFontSize = 12;
        imLabel.LabelFontColor = currentColor;
        imLabel.LabelBold = true;

        // ── Voltage chain head-to-tail ──
        double tipX = 0, tipY = 0;
        for (int i = 0; i < 5; i++)
        {
            var item = _vm.PhasorItemsIdealized[2 + i];
            double dx = item.Magnitude * Math.Cos(item.AngleRad);
            double dy = item.Magnitude * Math.Sin(item.AngleRad);
            double endX = tipX + dx;
            double endY = tipY + dy;
            var color = Color.FromHex(chainColors[i]);

            var arrow = plt.Add.Arrow(new Coordinates(tipX, tipY),
                new Coordinates(endX, endY));
            arrow.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
            arrow.ArrowLineColor = color;
            arrow.ArrowLineWidth = 2;
            arrow.ArrowheadLength = 8;
            arrow.ArrowheadWidth = 6;

            var txt = plt.Add.Text(item.Name, (tipX + endX) / 2 * 1.05,
                (tipY + endY) / 2 * 1.05);
            txt.LabelFontSize = 10;
            txt.LabelFontColor = color;

            tipX = endX;
            tipY = endY;
        }

        // ── Resultant U ──
        var uColor = Color.FromHex("#E74C3C");
        var uArrow = plt.Add.Arrow(new Coordinates(0, 0), new Coordinates(tipX, tipY));
        uArrow.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        uArrow.ArrowLineColor = uColor;
        uArrow.ArrowLineWidth = 2;
        uArrow.ArrowheadLength = 10;
        uArrow.ArrowheadWidth = 7;
        var uLabel = plt.Add.Text("Um", tipX * 1.08, tipY * 1.08);
        uLabel.LabelFontSize = 12;
        uLabel.LabelFontColor = uColor;
        uLabel.LabelBold = true;

        // ── Ud/Uq projections ──
        var projUdColor = Colors.DimGray;
        var projUd = plt.Add.Arrow(new Coordinates(0, 0), new Coordinates(tipX, 0));
        projUd.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        projUd.ArrowLineColor = projUdColor;
        projUd.ArrowLineWidth = 1;
        projUd.ArrowheadLength = 5;
        projUd.ArrowheadWidth = 4;
        var projUdLabel = plt.Add.Text("Ud", tipX * 0.6, 0.06 * lim);
        projUdLabel.LabelFontSize = 9;
        projUdLabel.LabelFontColor = projUdColor;

        var projUq = plt.Add.Arrow(new Coordinates(0, 0), new Coordinates(0, tipY));
        projUq.ArrowShape = new ScottPlot.ArrowShapes.SingleLine();
        projUq.ArrowLineColor = projUdColor;
        projUq.ArrowLineWidth = 1;
        projUq.ArrowheadLength = 5;
        projUq.ArrowheadWidth = 4;
        var projUqLabel = plt.Add.Text("Uq", 0.05 * lim, tipY * 0.65);
        projUqLabel.LabelFontSize = 9;
        projUqLabel.LabelFontColor = projUdColor;

        // ── Dotted construction lines from tip to axes ──
        var hLine = plt.Add.Scatter(new double[] { 0, tipX }, new double[] { tipY, tipY }, projUdColor);
        if (hLine is ScottPlot.Plottables.Scatter sh2)
        {
            sh2.LinePattern = LinePattern.Dotted;
            sh2.LineWidth = 0.8f;
            sh2.MarkerSize = 0;
        }
        var vLine = plt.Add.Scatter(new double[] { tipX, tipX }, new double[] { 0, tipY }, projUdColor);
        if (vLine is ScottPlot.Plottables.Scatter sv2)
        {
            sv2.LinePattern = LinePattern.Dotted;
            sv2.LineWidth = 0.8f;
            sv2.MarkerSize = 0;
        }

        PhasorPlotIdealized.Refresh();
    }

    // ═══════════════════════════════════════════════════════
    // PWM Output Voltage Time Domain (line-to-line)
    // ═══════════════════════════════════════════════════════
    private void InitPwmVoltageTimePlot()
    {
        var plt = VoltageTimePlot.Plot;
        ApplyWhiteStyle(plt);
        plt.Title("PWM Output Voltage (Line-to-Line)");
        plt.XLabel("Time (s)");
        plt.YLabel("Voltage (V)");
        plt.ShowLegend();
    }

    // ═══════════════════════════════════════════════════════
    // PWM Output Voltage FFT
    // ═══════════════════════════════════════════════════════
    private void InitPwmVoltageFFTPlot()
    {
        var plt = VoltageFFTPlot.Plot;
        ApplyWhiteStyle(plt);
        plt.Title("PWM Output Voltage FFT");
        plt.XLabel("Frequency (Hz)");
        plt.YLabel("Amplitude (V)");
        plt.ShowLegend();
    }

    // ═══════════════════════════════════════════════════════
    // Motor Voltage Time Domain (after LC filter)
    // ═══════════════════════════════════════════════════════
    private void InitMotorVoltageTimePlot()
    {
        var plt = MotorVoltageTimePlot.Plot;
        ApplyWhiteStyle(plt);
        plt.Title("Motor Voltage (Line-to-Line, after LC Filter)");
        plt.XLabel("Time (s)");
        plt.YLabel("Voltage (V)");
        plt.ShowLegend();
    }

    // ═══════════════════════════════════════════════════════
    // Motor Voltage FFT (after LC filter)
    // ═══════════════════════════════════════════════════════
    private void InitMotorVoltageFFTPlot()
    {
        var plt = MotorVoltageFFTPlot.Plot;
        ApplyWhiteStyle(plt);
        plt.Title("Motor Voltage FFT (after LC Filter)");
        plt.XLabel("Frequency (Hz)");
        plt.YLabel("Amplitude (V)");
        plt.ShowLegend();
    }

    // ═══════════════════════════════════════════════════════
    // Motor Current Time Domain
    // ═══════════════════════════════════════════════════════
    private void InitCurrentTimePlot()
    {
        var plt = CurrentTimePlot.Plot;
        ApplyWhiteStyle(plt);
        plt.Title("Motor Currents (Time Domain)");
        plt.XLabel("Time (s)");
        plt.YLabel("Current (A)");
        plt.ShowLegend();
    }

    // ═══════════════════════════════════════════════════════
    // Motor Current FFT
    // ═══════════════════════════════════════════════════════
    private void InitCurrentFFTPlot()
    {
        var plt = CurrentFFTPlot.Plot;
        ApplyWhiteStyle(plt);
        plt.Title("Motor Current FFT");
        plt.XLabel("Harmonic Order");
        plt.YLabel("Amplitude (A)");
        plt.ShowLegend();
    }

    private static void ApplyWhiteStyle(Plot plt)
    {
        plt.FigureBackground.Color = Colors.White;
        plt.DataBackground.Color = Colors.White;
        plt.Axes.Color(Colors.Black);
    }

    // ═══════════════════════════════════════════════════════
    // Shared Refresh for Time-Domain plots
    // ═══════════════════════════════════════════════════════
    private void RefreshTimeDomainPlots()
    {
        var t = _vm.TimeData;
        var vuv = _vm.VUV;
        var vvw = _vm.VVW;
        var vwu = _vm.VWU;
        var iu = _vm.IU;
        var iv = _vm.IV;
        var iw = _vm.IW;

        if (t.Length == 0) return;

        RefreshLinePlot(VoltageTimePlot, t, vuv, vvw, vwu, "V_UV", "V_VW", "V_WU", "Voltage (V)");
        RefreshLinePlot(CurrentTimePlot, t, iu, iv, iw, "I_U", "I_V", "I_W", "Current (A)");

        // Motor voltage (only populated when LC filter is active)
        var motorVuv = _vm.MotorVUV;
        var motorVvw = _vm.MotorVVW;
        var motorVwu = _vm.MotorVWU;
        if (motorVuv.Length > 0 || motorVvw.Length > 0 || motorVwu.Length > 0)
        {
            RefreshLinePlot(MotorVoltageTimePlot, t, motorVuv, motorVvw, motorVwu,
                "V_UV(motor)", "V_VW(motor)", "V_WU(motor)", "Voltage (V)");
        }
    }

    private static void RefreshLinePlot(ScottPlot.Avalonia.AvaPlot avaPlot,
        double[] t, double[] y1, double[] y2, double[] y3,
        string label1, string label2, string label3, string yLabel)
    {
        var plt = avaPlot.Plot;
        // Clear all plottables
        var plottables = plt.GetPlottables().ToList();
        foreach (var p in plottables)
            plt.Remove(p);

        plt.YLabel(yLabel);

        // Add line-only scatter plots (no markers)
        AddLineOnly(plt, t, y1, Color.FromHex("#E74C3C"), label1);
        AddLineOnly(plt, t, y2, Color.FromHex("#3498DB"), label2);
        AddLineOnly(plt, t, y3, Color.FromHex("#2ECC71"), label3);

        double maxAbs = MaxAbs3(y1, y2, y3) * 1.2;
        if (maxAbs < 1) maxAbs = 1;
        plt.Axes.SetLimits(t.First(), t.Last(), -maxAbs, maxAbs);
        plt.ShowLegend();
        avaPlot.Refresh();
    }

    private static void AddLineOnly(Plot plt, double[] xs, double[] ys, Color color, string label)
    {
        var scatter = plt.Add.Scatter(xs, ys, color);
        scatter.MarkerSize = 0;
        scatter.LineWidth = 1.5f;
        scatter.LegendText = label;
    }

    private static double MaxAbs3(double[] a, double[] b, double[] c)
    {
        double maxA = Math.Max(Math.Abs(a.Max()), Math.Abs(a.Min()));
        double maxB = Math.Max(Math.Abs(b.Max()), Math.Abs(b.Min()));
        double maxC = Math.Max(Math.Abs(c.Max()), Math.Abs(c.Min()));
        return Math.Max(maxA, Math.Max(maxB, maxC));
    }

    // ═══════════════════════════════════════════════════════
    // Shared Refresh for FFT plots
    // ═══════════════════════════════════════════════════════
    private void RefreshFFTPlots()
    {
        RefreshFFTPlot(VoltageFFTPlot, _vm.VUV_FFT, _vm.VVW_FFT, _vm.VWU_FFT,
            "V_UV", "V_VW", "V_WU", "Amplitude (V)");
        RefreshFFTPlot(CurrentFFTPlot, _vm.IU_FFT, _vm.IV_FFT, _vm.IW_FFT,
            "I_U", "I_V", "I_W", "Amplitude (A)");
    }

    private void RefreshMotorVoltagePlots()
    {
        RefreshFFTPlot(MotorVoltageFFTPlot, _vm.MotorVUV_FFT, _vm.MotorVVW_FFT, _vm.MotorVWU_FFT,
            "V_UV(motor)", "V_VW(motor)", "V_WU(motor)", "Amplitude (V)");
    }

    private void RefreshFFTPlot(ScottPlot.Avalonia.AvaPlot avaPlot,
        FFTContainer? fft1, FFTContainer? fft2, FFTContainer? fft3,
        string label1, string label2, string label3, string yLabel)
    {
        var plt = avaPlot.Plot;
        var plottables = plt.GetPlottables().ToList();
        foreach (var p in plottables)
            plt.Remove(p);

        plt.YLabel(yLabel);
        plt.XLabel("Frequency (Hz)");

        // Convert harmonic order to frequency (Hz): f = order × f_electrical
        double fBase = _vm.OperatingPoint?.ElectricalFreqHz ?? 50;
        double fSw = _vm.SwitchingFrequency;
        double fMax = 4.0 * fSw; // show up to 4× switching frequency

        DrawFFT(plt, fft1, fBase, fMax, "#E74C3C", label1);
        DrawFFT(plt, fft2, fBase, fMax, "#3498DB", label2);
        DrawFFT(plt, fft3, fBase, fMax, "#2ECC71", label3);

        // Set X limit to 4×Fsw
        plt.Axes.SetLimitsX(0, fMax);

        plt.ShowLegend();
        avaPlot.Refresh();
    }

    private static void DrawFFT(Plot plt, FFTContainer? fft, double fBase, double fMax,
        string hexColor, string label)
    {
        if (!fft.HasValue) return;
        var amp = fft.Value.Amplitude;
        var order = fft.Value.Order;
        if (amp == null || order == null) return;

        // Convert order to frequency, filter up to fMax
        var indices = new List<int>();
        var freqs = new List<double>();
        var amps = new List<double>();
        for (int i = 0; i < order.Count; i++)
        {
            double f = order[i] * fBase;
            if (f > fMax) break;
            // Skip DC component (order=0) and near-zero amplitudes for log scale
            double a = amp[i];
            if (a < 1e-12) continue;
            freqs.Add(f);
            amps.Add(a);
        }
        if (freqs.Count == 0) return;

        double[] xs = freqs.ToArray();
        double[] ys = amps.ToArray();
        var scatter = plt.Add.Scatter(xs, ys, Color.FromHex(hexColor));
        scatter.MarkerSize = 0;
        scatter.LegendText = label;
    }
}