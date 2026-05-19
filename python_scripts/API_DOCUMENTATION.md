# PMSM Drive Calculator — Python API Documentation

## Table of Contents

1. [Overview](#overview)
2. [Installation & Prerequisites](#installation--prerequisites)
3. [Architecture](#architecture)
4. [Quick Start](#quick-start)
5. [API Reference](#api-reference)
   - [`PMSMDriveCalcPython.__init__`](#pmsmdrivecalcpython__init__)
   - [`PMSMDriveCalcPython.run`](#pmsmdrivecalcpythonrun)
   - [`PMSMDriveCalcPython.print_result`](#pmsmdrivecalcpythonprint_result)
   - [`PMSMDriveCalcPython.export_waveforms_csv`](#pmsmdrivecalcpythonexport_waveforms_csv)
   - [`PMSMDriveCalcPython.export_fft_csv`](#pmsmdrivecalcpythonexport_fft_csv)
6. [Data Structures](#data-structures)
   - [`ComputationResult`](#computationresult)
   - [`OperatingPoint`](#operatingpoint)
   - [`FFTData`](#fftdata)
7. [CLI Reference](#cli-reference)
8. [Advanced Usage Examples](#advanced-usage-examples)
9. [Environment Variables](#environment-variables)
10. [Troubleshooting](#troubleshooting)

---

## Overview

`pmsm_drive_calc.py` is a Python wrapper around the **PMSMDriveCalc** .NET library. It uses [Python.NET](https://github.com/pythonnet/pythonnet) to call the compiled `.dll` **in-process** — no REST/HTTP server, no sockets, no serialisation overhead. The Python script directly instantiates .NET objects and invokes methods on them.

The wrapper mirrors the full pipeline of the GUI application:

```
Motor Data  →  PWM Strategy  →  (Optional LC Filter)  →  Solver  →  FFT Analysis  →  Result
```

All physical quantities use **SI units** (volts, amperes, henries, farads, webers, newton-metres, watts).

---

## Installation & Prerequisites

### 1. Build the .NET DLL

```bash
cd /path/to/PMSMDriveCalc
dotnet build PMSMDriveCalc/PMSMDriveCalc.csproj -c Release
```

The DLL is output to `PMSMDriveCalc/bin/Release/net9.0/PMSMDriveCalc.dll`.

### 2. Install Python dependencies

```bash
pip install pythonnet
```

> **macOS/Linux note:** The script auto-detects whether Mono is available. If not, it sets `PYTHONNET_RUNTIME=coreclr` and uses the .NET 9 runtime directly.

### 3. Verify NuGet packages are restored

The `Meta.Numerics` library (used for FFT computation) is required at runtime. The script finds it automatically from `~/.nuget/packages/meta.numerics/`. If missing, run:

```bash
dotnet restore PMSMDriveCalc/PMSMDriveCalc.csproj
```

---

## Architecture

```
┌──────────────────────────────────────────────────────┐
│                  pmsm_drive_calc.py                   │
│                                                      │
│  PMSMDriveCalcPython                                 │
│    ├── .__init__(poles, R, Ld, Lq, psi_pm, speed)    │
│    ├── .run(Vll, phase, fsw, vdc, mod, solver, ...)  │
│    │     ├── PMSMdq(motor)                            │
│    │     ├── SPWM2 / SVPWM2 / SVPWM3 / IPSPWM3       │
│    │     ├── (Optional) OutputLCFilter                │
│    │     ├── (Optional) DCFilteredPWM                 │
│    │     ├── PMSMDQDriveCalculator.Compute()          │
│    │     └── _build_result() → ComputationResult      │
│    ├── .print_result(result)                         │
│    ├── .export_waveforms_csv(result, filename)        │
│    └── .export_fft_csv(result, filename)              │
│                                                      │
│  Data classes (pure Python):                         │
│    OperatingPoint, FFTData, ComputationResult         │
└──────────────────────────────────────────────────────┘
         │ Python.NET (in-process CLR host)
         ▼
┌──────────────────────────────────────────────────────┐
│              PMSMDriveCalc.dll (.NET 9)               │
│                                                      │
│  PMSMdq, SPWM2, SVPWM2/3, IPSPWM3                    │
│  OutputLCFilter, GridRectifierFilter, DCFilteredPWM   │
│  DQTransientSolverStar (DQ 2×2 backward Euler)        │
│  DQTransientSolverWithLCFilterFull (DQ 6-state)       │
│  FFTOperations (Meta.Numerics)                       │
│  PMSMDriveResult, OperatingPointResult                │
└──────────────────────────────────────────────────────┘
```

### Key Design Decisions

| Aspect | Choice |
|--------|--------|
| Motor model | `PMSMdq` — dq-frame PMSM with constant Ld/Lq |
| Motor connection | **Y (star)** connected |
| LC filter topology | **Y-connected** capacitors, floating star point, 3-wire system |
| Voltage input | Peak line-to-line `Vll` → internally converted to `Ud/Uq` via `Vpn_peak = Vll / √3` |
| FFT | Computed by `FFTOperations.GetFFT()` using **Meta.Numerics** library |
| Time step | Automatically chosen by each solver (typically `1/fsw` for Transient) |

---

## Quick Start

```python
from pmsm_drive_calc import PMSMDriveCalcPython

# 8-pole PMSM, 3000 RPM
calc = PMSMDriveCalcPython(
    poles=8, R=0.15, Ld=0.0025, Lq=0.005, psi_pm=0.125, speed=3000
)

# SVPWM 2-level, Transient solver, 20 fundamental periods
result = calc.run(
    Vll=400,         # target line-to-line peak voltage
    phase_deg=151.5, # voltage angle in dq-frame
    fsw=8000,        # switching frequency
    vdc=400,         # DC-link voltage
    mod="SVPWM2",    # modulation strategy
    solver="Transient",
    periods=20,
)

# Human-readable summary
calc.print_result(result)

# CSV export
calc.export_waveforms_csv(result, "waveforms.csv")
calc.export_fft_csv(result, "fft.csv")
```

---

## API Reference

### `PMSMDriveCalcPython.__init__`

```python
PMSMDriveCalcPython(
    poles: int = 8,
    R: float = 0.15,
    Ld: float = 0.0025,
    Lq: float = 0.005,
    psi_pm: float = 0.125,
    speed: float = 3000,
    L_sigma_ratio: float = 0.1,
)
```

Stores motor parameters. These are reused across multiple `run()` calls — create one calculator instance and call `run()` with different voltage/solver configurations.

| Parameter | Type | Unit | Default | Description |
|-----------|------|------|---------|-------------|
| `poles` | `int` | — | `8` | Number of rotor poles (must be even). Electrical frequency = `speed/60 × poles/2`. |
| `R` | `float` | Ω | `0.15` | Stator phase resistance. Used for copper loss calculation and LC filter transfer function. |
| `Ld` | `float` | H | `0.0025` | d-axis inductance. Salient-pole machines have `Lq > Ld`; surface-mount PMSM have `Ld ≈ Lq`. |
| `Lq` | `float` | H | `0.005` | q-axis inductance. Together with Ld determines the reluctance torque component. |
| `psi_pm` | `float` | Wb | `0.125` | Permanent-magnet flux linkage. Product of magnet flux and effective turns. |
| `speed` | `float` | RPM | `3000` | Rotor mechanical speed. Used to compute back-EMF = `ω_e × ψ_pm`. |
| `L_sigma_ratio` | `float` | — | `0.1` | Leakage/d-axis inductance ratio (Lσ/Ld) (clamped to 0.0–0.5). Used for mutual inductance: `Lm = Ld × (1 − Lσ_ratio)`, `Lσ = Ld × Lσ_ratio`. |

---

### `PMSMDriveCalcPython.run`

```python
def run(
    self,
    Vll: float = 400.0,
    phase_deg: float = 151.5,
    fsw: float = 8000.0,
    vdc: float = 400.0,
    mod: str = "SVPWM2",
    third_harmonic: bool = False,
    solver: str = "Transient",
    periods: int = 20,
    lc_filter: bool = False,
    Lf: float = 0.0004,
    Cf: float = 0.000025,
    grid_filter: bool = False,
    grid_Vpk: float = 245.0,
    grid_freq: float = 50.0,
    dc_cap: float = 0.0035,
    dc_ind: float = 0.0005,
) -> ComputationResult
```

Executes the full PMSM drive simulation and returns a [`ComputationResult`](#computationresult).

#### Voltage & Inverter Parameters

| Parameter | Type | Unit | Default | Description |
|-----------|------|------|---------|-------------|
| `Vll` | `float` | V | `400.0` | **Target** line-to-line peak voltage. Internally converted: `Vpn_peak = Vll / √3`, then `Ud = Vpn_peak·cos(θ)`, `Uq = Vpn_peak·sin(θ)`. For SVPWM, clamped to `vdc` (linear modulation limit). |
| `phase_deg` | `float` | ° | `151.5` | Voltage phasor angle in the dq-frame: `θ = atan2(Uq, Ud)`. 151.5° gives `Ud ≈ -203`, `Uq ≈ +110` at `Vll=400` (typical MTPA region). |
| `fsw` | `float` | Hz | `8000.0` | PWM switching frequency. Higher values reduce current ripple but increase switching losses. |
| `vdc` | `float` | V | `400.0` | DC-link voltage. Determines maximum fundamental output: `V_ll_max = vdc` for SVPWM, `vdc × √3/2` for SPWM. |
| `mod` | `str` | — | `"SVPWM2"` | Modulation strategy. See [Modulation Types](#modulation-types) below. |
| `third_harmonic` | `bool` | — | `False` | Enable 1/6 third-harmonic injection (increases linear modulation range by ~15%). Only valid with `SPWM2` or `IPSPWM3`. |

**Modulation Types:**

| Key | Class | Levels | Third Harmonic | Linear Mod. Limit |
|-----|-------|--------|----------------|-------------------|
| `"SPWM2"` | `SPWM2` | 2 | Optional | `vdc × √3/2 ≈ 0.866 vdc` |
| `"IPSPWM3"` | `IPSPWM3` | 3 | Optional | `vdc × √3/2` (with 3rd) |
| `"SVPWM2"` | `SVPWM2`¹ | 2 | N/A | `vdc` (full DC-link) |
| `"SVPWM3"` | `SVPWM3`¹ | 3 | N/A | `vdc` |
| `"QuasiSVPWM2"` | `QuasiSVPWM2` | 2 | N/A | `vdc` |
| `"QuasiSVPWM3"` | `QuasiSVPWM3` | 3 | N/A | `vdc` |

> ¹ **ZOH phase-lag fix:** `"SVPWM2"` and `"SVPWM3"` are internally implemented via `QuasiSVPWM2` / `QuasiSVPWM3` (zero-sequence injection / saddle waveform). This is mathematically equivalent to traditional space-vector PWM but avoids the zero-order-hold (ZOH) phase lag of δθ = ω·Ts/2 (≈ 4.5° at 200 Hz, 8 kHz) introduced when the reference vector is sampled once per switching period in subdomain/sector-based SVPWM computation. The saddle waveform formula `u_zero = (max+min)/2` is subtracted from each phase, producing identical line-to-line voltages while allowing continuous reference evaluation at every sub-sample (200× per switching period). The original `SVPWM2`/`SVPWM3` classes (subdomain/sector-based, preserved in source) are also directly callable, but `"SVPWM2"` and `"SVPWM3"` redirect to the QuasiSVPWM variants.

#### Solver Parameters

| Parameter | Type | Unit | Default | Description |
|-----------|------|------|---------|-------------|
| `solver` | `str` | — | `"Transient"` | Numerical solver. See [Solver Types](#solver-types) below. |
| `periods` | `int` | — | `20` | Number of fundamental electrical periods to simulate. More periods = better FFT resolution but longer runtime. 20 periods at 200 Hz = 0.1 s. |

**Solver Types:**

| Key | .NET Enum | Accuracy | Speed | When to Use |
|-----|-----------|----------|-------|-------------|
| `"Transient"` | `SolverType.Transient` | ★★★★ | ★★★☆ | DQ 2×2 backward Euler; accurate for PWM ripple and transient effects |
| `"Transient+LC-Full"` | `SolverType.TransientWithLCFilterFull` | ★★★★ | ★★☆☆ | DQ 6-state fully coupled; required when LC output filter is active |
| `"Transient+LC"` | `SolverType.TransientWithLCFilter` | ★★★★ | ★★☆☆ | Legacy ABC 6-state LC filter solver (prefer `Transient+LC-Full`) |

> **Note:** The `"AC"` solver key is still accepted for backward compatibility but is internally redirected to `SolverType.Transient` (DQ 2×2 backward Euler with steady-state initial conditions). The `"Transient+LC"` key maps to the legacy ABC-frame 6-state solver; prefer `"Transient+LC-Full"` for the DQ-frame coupled solver.

> **Important:** When `lc_filter=True`, use `solver="Transient+LC-Full"` for the DQ 6-state coupled LC+motor solver, or `solver="Transient+LC"` for the legacy ABC 6-state solver.

#### LC Output Filter Parameters

| Parameter | Type | Unit | Default | Description |
|-----------|------|------|---------|-------------|
| `lc_filter` | `bool` | — | `False` | Enable Y-connected LC output filter between inverter and motor terminals. |
| `Lf` | `float` | H | `0.0004` | Filter inductance per phase (400 µH). Typical range: 100 µH – 1 mH. |
| `Cf` | `float` | F | `0.000025` | Filter capacitance per phase (25 µF), **Y-connected**. The floating star point means capacitor voltages are phase-to-neutral. |

**LC Filter Topology:**

```
Inverter ──┬── Lf ──┬── Motor Terminal U
           │        │
           │       Cf
           │        │
           │        └── (floating star point)
           │
           ├── Lf ──┬── Motor Terminal V
           │        │
           │       Cf
           │        │
           │        └── (floating star point)
           │
           └── Lf ──┬── Motor Terminal W
                    │
                   Cf
                    │
                    └── (floating star point)
```

When the LC filter is active, `ComputationResult` includes additional fields:
- `MotorVU`, `MotorVV`, `MotorVW` — motor terminal phase voltages (after filter)
- `MotorVUV`, `MotorVVW`, `MotorVWU` — motor line-to-line voltages
- `MotorVPhaseNeutralRms` — RMS phase-to-neutral voltage at motor
- `MotorVPhasePhaseFundamental` — fundamental line-to-line peak at motor
- FFT of all motor-side voltages

#### Grid DC-Link Ripple Filter Parameters

| Parameter | Type | Unit | Default | Description |
|-----------|------|------|---------|-------------|
| `grid_filter` | `bool` | — | `False` | Enable grid-side diode-bridge rectifier DC-link ripple simulation. |
| `grid_Vpk` | `float` | V | `245.0` | Grid phase voltage amplitude (`230 Vrms × √2 ≈ 245 V`). |
| `grid_freq` | `float` | Hz | `50.0` | Grid frequency (50 Hz for Europe/Asia, 60 Hz for Americas). |
| `dc_cap` | `float` | F | `0.0035` | DC-link capacitance (3500 µF). Affects ripple magnitude. |
| `dc_ind` | `float` | H | `0.0005` | DC-link series inductance (500 µH). Affects ripple shape and conduction angle. |

> **Note:** The grid filter models a **passive diode-bridge rectifier** (6-pulse). It adds 300/360 Hz ripple (6× grid frequency) to the DC-link voltage, which propagates through the PWM to the motor.

**Return Value:** [`ComputationResult`](#computationresult)

---

### `PMSMDriveCalcPython.print_result`

```python
@staticmethod
def print_result(r: ComputationResult) -> None
```

Prints a formatted summary to stdout:

```
────────────────────────────────────────────────────────────
  PMSM Drive Calculation Result
────────────────────────────────────────────────────────────

  ── Motor ──
  Poles              : 8
  Speed              : 3000.0 RPM
  Electrical freq    : 200.00 Hz
  ...

  ── Voltage Input (target) ──
  V_LL peak          : 400.00 V
  ...

  ── Currents ──
  Id                 : -16.49 A
  Iq                 : 31.86 A
  ...

  ── Torque & Power ──
  Torque             : 31.78 Nm
  Mechanical power   : 9983.88 W
  ...
```

### `PMSMDriveCalcPython.export_waveforms_csv`

```python
@staticmethod
def export_waveforms_csv(r: ComputationResult, filename: str) -> None
```

Exports time-domain waveforms to CSV.

**Columns:** `time, VU, VV, VW, VUV, VVW, VWU, IU, IV, IW`
(Plus `MotorVU, MotorVV, MotorVW, MotorVUV, MotorVVW, MotorVWU` when LC filter is active.)

### `PMSMDriveCalcPython.export_fft_csv`

```python
@staticmethod
def export_fft_csv(r: ComputationResult, filename: str) -> None
```

Exports FFT spectra to CSV.

**Columns:** `order, IU_amp, IU_phase_deg, IV_amp, IV_phase_deg, IW_amp, IW_phase_deg, VU_amp, VU_phase_deg`

- `order` — harmonic order relative to fundamental (1 = fundamental)
- `amp` — amplitude in physical units (A for currents, V for voltages)
- `phase_deg` — phase in degrees

---

## Data Structures

### `ComputationResult`

```python
@dataclass
class ComputationResult:
    op: OperatingPoint                    # Steady-state operating point
    time: List[float]                     # Time vector (seconds)
    
    # PWM phase voltages (inverter output)
    VU: List[float]                       # Phase U (V)
    VV: List[float]                       # Phase V (V)
    VW: List[float]                       # Phase W (V)
    
    # Line-to-line PWM voltages
    VUV: List[float]                      # V_U - V_V
    VVW: List[float]                      # V_V - V_W
    VWU: List[float]                      # V_W - V_U
    
    # Phase currents
    IU: List[float]                       # Phase U current (A)
    IV: List[float]                       # Phase V current (A)
    IW: List[float]                       # Phase W current (A)
    
    # Motor-side voltages (None unless LC filter active)
    MotorVU: Optional[List[float]]
    MotorVV: Optional[List[float]]
    MotorVW: Optional[List[float]]
    MotorVUV: Optional[List[float]]
    MotorVVW: Optional[List[float]]
    MotorVWU: Optional[List[float]]
    
    # FFT spectra (None if not computed)
    VU_FFT: Optional[FFTData]
    ... (FFT for all voltage/current waveforms)
    
    # Summary values
    BaseFrequencyHz: float                # Fundamental frequency
    PwmVPhasePhaseFundamental: float      # PWM V_LL fundamental peak
    MotorVPhaseNeutralRms: Optional[float] # Motor V_PN RMS (LC filter only)
    MotorVPhasePhaseFundamental: Optional[float] # Motor V_LL fundamental peak
```

### `OperatingPoint`

```python
@dataclass
class OperatingPoint:
    # ── Target (input) voltages ──
    Ud: float                             # d-axis voltage (V)
    Uq: float                             # q-axis voltage (V)
    PhaseVoltageMagnitude: float          # |V_dq| = √(Ud² + Uq²) (V)
    VoltageAngleRad: float                # atan2(Uq, Ud) (rad)
    
    # ── Steady-state currents ──
    Id: float                             # d-axis current (A)
    Iq: float                             # q-axis current (A)
    CurrentMagnitude: float               # |I_dq| = √(Id² + Iq²) (A)
    CurrentAngleRad: float                # atan2(Iq, Id) (rad)
    
    # ── Torque & Power ──
    TorqueNm: float                       # Electromagnetic torque (N·m)
    MechanicalPowerW: float               # Mechanical output power (W)
    ActivePowerW: float                   # Active electrical power (W)
    ApparentPowerVA: float                # Inverter apparent power (VA)
    PwmApparentPowerVA: float             # PWM-side apparent power (VA)
    PowerFactor: float                    # cos(φ) = Active/Apparent
    CopperLossW: float                    # I²R losses (W)
    
    # ── Motor terminal (actual, after filters) ──
    MotorUd: float                        # Motor terminal Ud (V)
    MotorUq: float                        # Motor terminal Uq (V)
    
    # ── Motor parameters (echoed) ──
    SpeedRPM: float
    ElectricalFreqHz: float
    ElectricalSpeedRadS: float
    Poles: int
    PhaseResistance: float
    Ld: float
    Lq: float
    PMFluxLinkage: float
```

### `FFTData`

```python
@dataclass
class FFTData:
    orders: List[int]                     # Harmonic order (1 = fundamental)
    amplitudes: List[float]               # Amplitude (A or V)
    phases_deg: List[float]               # Phase angle (degrees)
```

**Usage example:**

```python
result = calc.run(...)
if result.IU_FFT:
    for order, amp, phase in zip(
        result.IU_FFT.orders,
        result.IU_FFT.amplitudes,
        result.IU_FFT.phases_deg,
    ):
        if order <= 5:  # Show first 5 harmonics
            print(f"  IU harmonic {order}: {amp:.4f} A @ {phase:.1f}°")
```

---

## CLI Reference

```bash
python pmsm_drive_calc.py [OPTIONS]
```

### Motor Parameters

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--poles` | int | 8 | Number of rotor poles |
| `--R` | float | 0.15 | Phase resistance (Ω) |
| `--Ld` | float | 0.0025 | d-axis inductance (H) |
| `--Lq` | float | 0.005 | q-axis inductance (H) |
| `--psi-pm` | float | 0.125 | PM flux linkage (Wb) |
| `--speed` | float | 3000 | Rotor speed (RPM) |

### Voltage / Inverter

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--Vll` | float | 400.0 | Target line-to-line peak voltage (V) |
| `--phase` | float | 151.5 | Voltage angle in dq-frame (°) |
| `--fsw` | float | 8000.0 | Switching frequency (Hz) |
| `--vdc` | float | 400.0 | DC-link voltage (V) |
| `--mod` | str | SVPWM2 | Modulation: `SPWM2`, `IPSPWM3`, `SVPWM2`, `SVPWM3`, `QuasiSVPWM2`, `QuasiSVPWM3`. Note: `SVPWM2`/`SVPWM3` are internally implemented via QuasiSVPWM (zero-sequence injection, avoids ZOH phase lag). |
| `--third-harmonic` | flag | False | Enable 1/6 third-harmonic injection |

### Solver

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--solver` | str | Transient | `AC` (redirected), `Transient`, `Transient+LC`, `Transient+LC-Full` |
| `--periods` | int | 20 | Fundamental periods to simulate |

### LC Output Filter

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--lc-filter` | flag | False | Enable Y-connected LC filter |
| `--Lf` | float | 0.0004 | Filter inductance per phase (H) |
| `--Cf` | float | 0.000025 | Filter capacitance per phase (F) |

### Grid DC-Link Ripple Filter

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--grid-filter` | flag | False | Enable diode rectifier DC ripple |
| `--grid-Vpk` | float | 245.0 | Grid phase voltage amplitude (V) |
| `--grid-freq` | float | 50.0 | Grid frequency (Hz) |
| `--dc-cap` | float | 0.0035 | DC-link capacitance (F) |
| `--dc-ind` | float | 0.0005 | DC-link series inductance (H) |

### Output

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--export-csv` | str | — | CSV path for time-domain waveforms |
| `--export-fft-csv` | str | — | CSV path for FFT spectra |
| `--quiet` | flag | False | Suppress summary printout |

---

## Advanced Usage Examples

### Example 1: Compare All Modulation Strategies

```python
from pmsm_drive_calc import PMSMDriveCalcPython

calc = PMSMDriveCalcPython(poles=8, R=0.15, Ld=0.0025, Lq=0.005, psi_pm=0.125, speed=3000)

# All 6 strategies: SVPWM2/SVPWM3 internally use QuasiSVPWM (zero-sequence injection)
# which is mathematically equivalent but avoids ZOH phase lag (δθ = ω·Ts/2).
for mod in ["SPWM2", "IPSPWM3", "SVPWM2", "SVPWM3", "QuasiSVPWM2", "QuasiSVPWM3"]:
    result = calc.run(mod=mod, solver="Transient")
    print(f"{mod}: Torque={result.op.TorqueNm:.3f} Nm, "
          f"Id={result.op.Id:.3f} A, Iq={result.op.Iq:.3f} A, "
          f"PF={result.op.PowerFactor:.3f}")
```

### Example 2: LC Filter Parameter Sweep

```python
import csv
from pmsm_drive_calc import PMSMDriveCalcPython

calc = PMSMDriveCalcPython(poles=8, R=0.15, Ld=0.0025, Lq=0.005, psi_pm=0.125, speed=3000)

# Sweep filter capacitance
with open("lc_sweep.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Cf_uF", "TorqueNm", "Id", "Iq", "MotorV_PN_RMS",
                      "IU_THD_pct"])

    for Cf in [5e-6, 10e-6, 25e-6, 50e-6, 100e-6]:
        result = calc.run(
            mod="SVPWM2", solver="Transient+LC-Full",
            lc_filter=True, Cf=Cf, Lf=0.0004,
        )
        # Compute THD from FFT
        if result.IU_FFT:
            fundamental = result.IU_FFT.amplitudes[0]  # order 1
            harmonics_sq = sum(
                a**2 for o, a in zip(result.IU_FFT.orders,
                                     result.IU_FFT.amplitudes)
                if o > 1
            )
            thd = (harmonics_sq**0.5 / fundamental) * 100
        else:
            thd = 0.0

        writer.writerow([
            Cf * 1e6,
            result.op.TorqueNm,
            result.op.Id,
            result.op.Iq,
            result.MotorVPhaseNeutralRms,
            thd,
        ])
```

### Example 3: Grid Ripple Analysis

```python
from pmsm_drive_calc import PMSMDriveCalcPython

calc = PMSMDriveCalcPython(poles=8, R=0.15, Ld=0.0025, Lq=0.005, psi_pm=0.125, speed=3000)

# 50 Hz grid — 300 Hz ripple on DC-link
result = calc.run(
    mod="SVPWM2", solver="Transient",
    grid_filter=True,
    grid_Vpk=325.0,   # 230 Vrms × √2
    grid_freq=50.0,
    dc_cap=0.002,     # 2000 µF (more ripple)
    dc_ind=0.001,     # 1 mH
)

# The VU waveform now contains 300 Hz envelope modulation
# Analyze low-frequency content via FFT
if result.VU_FFT:
    for order, amp in zip(result.VU_FFT.orders, result.VU_FFT.amplitudes):
        if 0.1 < order < 10:
            print(f"  VU order {order:.2f}: {amp:.3f} V")
```

### Example 4: Speed-Torque Characteristic

```python
from pmsm_drive_calc import PMSMDriveCalcPython
import csv

calc = PMSMDriveCalcPython(poles=8, R=0.15, Ld=0.0025, Lq=0.005, psi_pm=0.125)

with open("speed_torque.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["SpeedRPM", "TorqueNm", "PowerW", "Id", "Iq", "PF"])

    for speed in range(500, 5001, 500):
        calc._speed = speed
        try:
            result = calc.run(
                Vll=400, phase_deg=151.5,
                mod="SVPWM2", solver="Transient",
            )
            writer.writerow([
                speed, result.op.TorqueNm, result.op.MechanicalPowerW,
                result.op.Id, result.op.Iq, result.op.PowerFactor,
            ])
        except Exception as e:
            print(f"Speed {speed} RPM failed: {e}")
```

### Example 5: Direct .NET Type Access (Advanced)

```python
# You can bypass the wrapper and use .NET types directly
import sys, os
if "" in sys.path:
    sys.path.remove("")

# Load assemblies (same setup code as pmsm_drive_calc.py)
sys.path.insert(0, "/path/to/PMSMDriveCalc/bin/Release/net9.0")
import clr
from System.Reflection import Assembly
Assembly.LoadFile("/path/to/Meta.Numerics.dll")
clr.AddReference("Meta.Numerics")
Assembly.LoadFile("/path/to/PMSMDriveCalc.dll")
clr.AddReference("PMSMDriveCalc")

from PMSMDriveCalc import PMSMdq, SVPWM2, PMSMDQDriveCalculator, SolverType
from System import Math

motor = PMSMdq(8, 0.125, 0.0025, 0.005, 0.15)
motor.SpeedRPM = 3000.0

pwm = SVPWM2(8000.0, 400.0)
calc = PMSMDQDriveCalculator(motor, pwm)

Vpn_peak = 400.0 / Math.Sqrt(3.0)
phase_rad = 151.5 * Math.PI / 180.0
Ud = Vpn_peak * Math.Cos(phase_rad)
Uq = Vpn_peak * Math.Sin(phase_rad)

dr = calc.Compute(Ud, Uq, SolverType.Transient, 20)

# Access .NET properties directly
print(f"Id = {dr.OperatingPoint.Id} A")
print(f"Torque = {dr.OperatingPoint.TorqueNm} Nm")
print(f"Time samples = {dr.Time.Count}")
```

---

## Environment Variables

| Variable | Description |
|----------|-------------|
| `PYTHONNET_PMSM_DLL` | Full path to `PMSMDriveCalc.dll`. Overrides auto-detection. Useful for custom build locations. |
| `PYTHONNET_RUNTIME` | Set to `coreclr` to force .NET runtime (default on macOS without Mono). Set to `mono` to force Mono runtime. |

**Example:**
```bash
export PYTHONNET_PMSM_DLL=/custom/path/PMSMDriveCalc.dll
export PYTHONNET_RUNTIME=coreclr
python pmsm_drive_calc.py
```

---

## Troubleshooting

### `ImportError: No module named 'clr'`

```bash
pip install pythonnet
```

### `Could not load file or assembly 'Meta.Numerics'`

The Meta.Numerics NuGet package wasn't restored. Run:

```bash
dotnet restore PMSMDriveCalc/PMSMDriveCalc.csproj
```

If the NuGet cache is in a non-standard location, copy `Meta.Numerics.dll` next to `PMSMDriveCalc.dll`:

```bash
find ~/.nuget -name "Meta.Numerics.dll" -exec cp {} PMSMDriveCalc/bin/Release/net9.0/ \;
```

### `ModuleNotFoundError` at `from PMSMDriveCalc import ...`

This happens when you run from the project root directory — Python finds the `PMSMDriveCalc/` source folder before the CLR-loaded types. The script now removes `""` from `sys.path` to prevent this. If you still hit this, run from a different directory:

```bash
cd /tmp
python /path/to/pmsm_drive_calc.py
```

### `PYTHONNET_RUNTIME=coreclr` not working on Linux

Ensure the .NET 9 runtime is installed:

```bash
dotnet --list-runtimes | grep 9.0
```

If missing, install from https://dotnet.microsoft.com/download/dotnet/9.0

### Computation hangs or is very slow

- Reduce `periods` (e.g., from 20 to 5 for quick tests)
- `solver="AC"` is internally redirected to `Transient` (same accuracy as DQ 2×2 solver)
- Large LC filter capacitance values require very small time steps

### Results differ from GUI app

- Check that all parameters match exactly (especially `Vll` vs `Vpn_peak`)
- The GUI clamps `Vll` to `vdc` for SVPWM — the Python wrapper does the same via the .NET library
- Ensure the solver type matches: LC filter requires `Transient+LC-Full` (DQ 6-state) or `Transient+LC` (legacy ABC 6-state)
