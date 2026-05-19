# PMSM Drive Calculator

A design and analysis tool for permanent-magnet synchronous motor (PMSM) drive systems.
Built with C# (.NET 9.0), Avalonia UI, and Python bridge for AI/LLM integration.

## Project Overview

Calculates the complete drive simulation pipeline:

1. **Steady-state dq solve** — Solve Id, Iq from target Ud/Uq with the dq voltage equations
2. **PWM modulation** — Generate 3-phase switching waveforms (SPWM2, IPSPWM3, QuasiSVPWM2/3 via zero-sequence injection)
3. **Time-domain transient solve** — DQ-frame backward Euler integration (2×2 motor, 6-state coupled LC+motor)
4. **FFT spectral analysis** — Harmonic spectra of phase voltages and currents
5. **Operating point computation** — Torque, power, efficiency, phasor diagrams

## Project Structure

```
PMSMDriveCalc/
├── PMSMDriveCalc.sln                         # Solution file
├── PMSMDriveCalc/                            # Core computation library
│   ├── PMSMDriveCalc.csproj
│   ├── PMSM.cs                               # PMSM dq-frame motor model (PMSMdq)
│   ├── DQDriveCalculator.cs                  # Steady-state solver + drive simulation pipeline
│   ├── Solver.cs                             # DQ-frame transient solvers + SolverType enum
│   ├── SPWM.cs                               # SPWM2, IPSPWM3, QuasiSVPWM2/3 modulation
│   ├── SVPWM.cs                              # SVPWM2, SVPWM3 space-vector modulation
│   ├── LCFilter.cs                           # Y-connected LC output filter
│   ├── GridFilter.cs                         # Grid-side DC-link ripple & AFE filter
│   ├── Miscellaneous.cs                      # FFT, geometry, list operations
│   └── Properties/
├── PMSMDriveCalc.UI/                         # Avalonia UI desktop application
│   ├── PMSMDriveCalc.UI.csproj
│   ├── App.axaml / App.axaml.cs
│   ├── Program.cs
│   ├── ViewModels/MainViewModel.cs            # MVVM view model (source-generated)
│   └── Views/
│       ├── MainWindow.axaml                   # UI layout
│       └── MainWindow.axaml.cs                # Code-behind + help/info content
├── PMSMDriveSim/                             # Legacy project (maintained for compatibility)
│   └── PMSMDriveSim.csproj
├── python_scripts/                           # Python bridge (Python.NET)
│   ├── pmsm_drive_calc.py                    # Python wrapper for PMSMDQDriveCalculator
│   ├── pmsm_tools.py                         # LLM tool definitions
│   └── API_DOCUMENTATION.md                  # Python API reference
├── docs/
│   ├── dq-solver-migration-design.md          # DQ-frame migration design document
│   └── transient-lc-filter-solver-design.md   # LC filter solver derivation
└── verify_lc/                                # LC filter verification project
```

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    GUI (Avalonia UI)                      │
│                 MainViewModel / MainWindow                │
├──────────────────────────────────────────────────────────┤
│               PMSMDQDriveCalculator                      │
│           (Ud, Uq → steady-state → PWM → solver → FFT)   │
├──────────────────────┬───────────────────────────────────┤
│   DQ Steady-State    │         PWM Modulation Layer       │
│  DQSteadyStateSolver  │  SPWM2 / IPSPWM3 / QuasiSVPWM2/QuasiSVPWM3 │
│  (matrix inversion)   │  DCFilteredPWM (decorator)       │
│                       │  OutputLCFilter (decorator)       │
├──────────────────────┴───────────────────────────────────┤
│              DQ-Frame Transient Solvers                    │
│  DQTransientSolverStar (2×2 backward Euler, no LC)        │
│  DQTransientSolverWithLCFilterFull (6-state coupled)      │
├──────────────────────────────────────────────────────────┤
│                   Motor Model Layer                        │
│       PMSMdq (Ld, Lq, ψPM, R, Lσ_ratio)                  │
│   [Legacy] PMSMWithConstantInductance / VariableInductance │
├──────────────────────────────────────────────────────────┤
│                   Utility Layer                            │
│      FFTOperations / Point / ListOperations                │
└──────────────────────────────────────────────────────────┘
```

### Design Patterns

| Pattern | Applied at | Description |
|---------|-----------|-------------|
| **Strategy** | `PWM` base class | Interchangeable PWM modulation strategies |
| **Decorator** | `DCFilteredPWM`, `OutputLCFilter` | Layer filtering/ripple on any PWM output |
| **MVVM** | `MainViewModel` + `MainWindow` | Source-generated observable properties & commands |

## Core Modules

### 1. Motor Model ([`PMSM.cs`](PMSMDriveCalc/PMSM.cs))

**`PMSMdq`** — Primary model operating in the rotor-synchronous dq reference frame:

| Parameter | Description |
|-----------|-------------|
| `Poles` | Number of poles (must be even) |
| `PMFluxLinkage` (ψPM) | Permanent-magnet flux linkage (Wb) |
| `DInductance` (Ld) | d-axis inductance (H) |
| `QInductance` (Lq) | q-axis inductance (H) |
| `PhaseResistance` (Rs) | Stator phase resistance (Ω) |
| `LSigmaRatio` | Leakage inductance ratio (Lσ = Lσ_ratio × Ld) |
| `SpeedRPM` | Rotor speed — derives `BaseFrequency`, `AngularSpeed` |

Steady-state voltage equations:
```
Ud = Rs·Id − ω·Lq·Iq
Uq = Rs·Iq + ω·Ld·Id + ω·ψPM
```

Torque:
```
Te = 1.5 × (poles/2) × [ψPM·Iq + (Ld − Lq)·Id·Iq]
```

### 2. PWM Modulation ([`SPWM.cs`](PMSMDriveCalc/SPWM.cs), [`SVPWM.cs`](PMSMDriveCalc/SVPWM.cs))

| Class | Levels | Description |
|-------|--------|-------------|
| `SPWM2` | 2-level | Sinusoidal PWM, optional 1/6 third-harmonic injection |
| `IPSPWM3` | 3-level | In-Phase Sinusoidal PWM, dual carrier, optional 3rd-harmonic injection |
| `SVPWM2` | 2-level | Space-vector PWM, 6 sectors, 7-segment symmetric pattern (redirected to QuasiSVPWM2) |
| `SVPWM3` | 3-level | NPC inverter SVPWM, 6 sectors × 4 sub-regions (redirected to QuasiSVPWM3) |
| `QuasiSVPWM2` | 2-level | Zero-sequence injection / saddle waveform — mathematically equivalent to SVPWM, avoids ZOH phase lag |
| `QuasiSVPWM3` | 3-level | Same for 3-level — avoids ZOH phase lag (δθ = ω·Ts/2 ≈ 4.5° at 200 Hz, 8 kHz) |

### 3. DQ-Frame Transient Solvers ([`Solver.cs`](PMSMDriveCalc/Solver.cs))

All solvers operate in the dq synchronous reference frame. The ABC→DQ conversion uses a
**zero-sequence-rejecting Clarke transform**:

```
v_α = (2·Va − Vb − Vc) / 3      ← cancels common-mode (e.g., 3rd harmonic)
v_β = (Vb − Vc) / √3
```

This prevents non-physical 3rd harmonic currents when SPWM third-harmonic injection is active.

#### `DQTransientSolverStar` — DQ 2×2 Backward Euler

For systems **without** LC output filter. Discretizes the dq voltage equations:

```
┌ 1+dt·R/Ld   −dt·ω·Lq/Ld ┐ ┌ id[k] ┐   ┌ id[k−1] + dt·vd/Ld           ┐
└ dt·ω·Ld/Lq   1+dt·R/Lq  ┘ └ iq[k] ┘ = └ iq[k−1] + dt·(vq−ω·ψPM)/Lq ┘
```

Solved via Cramer's rule. Per-step cost: O(1), ~12 multiplications.

#### `DQTransientSolverWithLCFilterFull` — DQ 6-State Coupled

For systems **with** LC output filter. Integrates filter and motor as one coupled system:

| State | Description |
|-------|-------------|
| `i_Lf_d`, `i_Lf_q` | Filter inductor currents in dq frame |
| `v_Cf_d`, `v_Cf_q` | Filter capacitor voltages (= motor terminal voltages) |
| `i_m_d`, `i_m_q` | Motor phase currents in dq frame |

Per-step algorithm (sequential semi-implicit):
1. **Step A** — Motor 2×2 backward Euler solve with v_Cf[k−1] (predictor)
2. **Step B** — Capacitor update (explicit Euler for cross terms)
3. **Step C** — Inductor update (explicit Euler for cross terms)
4. **Step D** — Corrector: re-solve motor 2×2 with updated v_Cf[k]

Per-step cost: O(1), ~30 multiplications.

### 4. Drive Calculator ([`DQDriveCalculator.cs`](PMSMDriveCalc/DQDriveCalculator.cs))

`PMSMDQDriveCalculator` orchestrates the full pipeline:

```csharp
var calc = new PMSMDQDriveCalculator(motor, pwm);
var result = calc.Compute(Ud, Uq, SolverType.Transient, periods);
// result.Time, result.IU/IV/IW, result.VU/VV/VW, result.IU_FFT, etc.
// result.OperatingPoint — steady-state Id, Iq, torque, power, etc.
```

Pipeline:
1. Steady-state solve → Id, Iq, torque, phase voltage magnitude/angle
2. Inverse Park + Clarke → 3-phase sinusoidal references
3. PWM modulation → switching voltage waveforms
4. Transient solver → time-domain motor currents
5. FFT analysis → harmonic spectra of voltages and currents

### 5. Filters

| File | Class | Function |
|------|-------|----------|
| [`LCFilter.cs`](PMSMDriveCalc/LCFilter.cs) | `OutputLCFilter` | Y-connected LC low-pass filter (Lf per phase, Cf Y-connected). In DQ-frame path, the `DQTransientSolverWithLCFilterFull` handles the coupled dynamics. |
| [`GridFilter.cs`](PMSMDriveCalc/GridFilter.cs) | `GridRectifierFilter` | Grid → diode rectifier → DC link (6ω ripple) |
| [`GridFilter.cs`](PMSMDriveCalc/GridFilter.cs) | `DCFilteredPWM` | Decorator injecting DC-link ripple into PWM output |
| [`GridFilter.cs`](PMSMDriveCalc/GridFilter.cs) | `AFEGridFilter` | Active front-end + LCL filter simplified model |

## Mathematical Summary

### Clarke Transform (Zero-Sequence-Rejecting)

```
v_α = (2·Vu − Vv − Vw) / 3
v_β = (Vv − Vw) / √3
```

This form inherently rejects common-mode (V₀) because Σ(2U−V−W) = 0 for any equal triple.
Critical when SPWM 3rd harmonic injection is active — the 3rd harmonic is pure common-mode.

### Park Transform (αβ → dq)

```
vd =  vα·cos(θ) + vβ·sin(θ)
vq = −vα·sin(θ) + vβ·cos(θ)
```

Angle θ is propagated via trigonometric recurrence: `sin(θ+Δθ) = sinθ·cosΔθ + cosθ·sinΔθ`,
avoiding expensive `Math.Sin/Cos` calls per time step.

### Inverse Park + Clarke (dq → abc)

```
iα =  id·cos(θ) − iq·sin(θ)
iβ =  id·sin(θ) + iq·cos(θ)

iu = iα
iv = −iα/2 + (√3/2)·iβ
iw = −iα/2 − (√3/2)·iβ
```

## GUI Application

The Avalonia UI desktop app provides:

- **Motor tab** — Ld, Lq, ψPM, Rs, Lσ_ratio, pole pairs, speed
- **Inverter tab** — DC-link voltage, switching frequency, modulation strategy, target Ud/Uq
- **Solver tab** — Solver type selection (auto-adapts to LC filter state)
- **Results** — Operating point table (idealized vs. actual), time-domain plots, FFT spectra, phasor diagrams
- **Info & Help dialogs** — Selectable/copyable text with detailed mathematical background
- **Sync demodulation** — IdFund/IqFund extracted via DC-mean of id[k], iq[k] over steady-state window (immune to FFT spectral leakage)

Build and run:
```bash
cd PMSMDriveCalc.UI
dotnet run
```

## Python Bridge

The [`python_scripts/`](python_scripts/) directory provides Python access to the C# computation engine via Python.NET:

```python
from pmsm_drive_calc import PMSMDriveCalcPython

calc = PMSMDriveCalcPython(poles=8, R=0.15, Ld=0.0025, Lq=0.005,
                           psi_pm=0.125, speed=3000)
result = calc.run(Vll=400.0, phase_deg=151.5, fsw=8000.0, vdc=400.0,
                  mod="SVPWM2", solver="Transient", periods=20)
calc.print_result(result)
```

Tool definitions for AI/LLM function calling are in [`pmsm_tools.py`](python_scripts/pmsm_tools.py).

## Dependencies

| Package | Purpose |
|---------|---------|
| `Avalonia UI` 11.x | Cross-platform .NET UI framework |
| `ScottPlot` 5.x | Interactive charting |
| `CommunityToolkit.Mvvm` | MVVM source generators |
| `Meta.Numerics` | FFT computation (`FourierTransformer`) |
| `System.Numerics` | Complex number arithmetic |
| `FodyWeavers` | IL weaving (compile-time code injection) |

## License

MIT License

Copyright (c) 2025

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
