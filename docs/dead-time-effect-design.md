# Dead-Time Effect — Design & Integration Document

## Table of Contents

1. [Physical Background](#1-physical-background)
2. [Mathematical Formulation](#2-mathematical-formulation)
3. [Architecture & Integration](#3-architecture--integration)
4. [Steady-State Current Approximation](#4-steady-state-current-approximation)
5. [Implementation Details](#5-implementation-details)
6. [Expected Harmonic Signatures](#6-expected-harmonic-signatures)
7. [Stacking Order & Decorator Chain](#7-stacking-order--decorator-chain)
8. [Usage Examples](#8-usage-examples)
9. [Limitations & Assumptions](#9-limitations--assumptions)
10. [GUI & Python API Surface](#10-gui--python-api-surface)

---

## 1. Physical Background

### 1.1 What is Dead-Time?

In a voltage-source inverter (VSI), each leg consists of two switches (upper and lower)
connected in series across the DC-link. Turning both on simultaneously would create a
**shoot-through** short circuit, destroying the switches instantly.

To prevent this, a **blanking interval** (dead-time, $$t_d$$) is inserted between turning
off one switch and turning on its complement. During $$t_d$$, both switches are off, and
the output voltage of that leg is **not controlled** by the gate signals — it is determined
by the direction of the phase current through the freewheeling diodes.

### 1.2 Voltage Error Mechanism

Consider one inverter leg (phase U):

- When $$i_U > 0$$ (current flowing **out** of the inverter): the lower freewheeling diode
  conducts during dead-time, pulling the output to the negative DC rail ($$-V_{dc}/2$$
  referenced to DC midpoint). The leg loses positive voltage-time area.
- When $$i_U < 0$$ (current flowing **into** the inverter): the upper freewheeling diode
  conducts, pulling the output to $$+V_{dc}/2$$. The leg gains positive voltage-time area.

The **average voltage error per switching period** $$T_s$$ is:

$$\Delta V_{err,phase} = -\operatorname{sign}(i_{phase}) \cdot \frac{t_d}{T_s} \cdot V_{dc}$$

This is a **non-linear** disturbance: it depends on the sign of the phase current, which
in turn depends on the motor operating point and the PWM voltage itself.

### 1.3 Characteristic Harmonic Signature

The sign function `sgn(sin(θ))` applied to a sinusoidal current produces a **square wave**
at the fundamental frequency. Fourier decomposition of this square wave yields:

$$\operatorname{sign}(\sin \omega t) = \frac{4}{\pi} \left( \sin \omega t + \frac{1}{3} \sin 3\omega t + \frac{1}{5} \sin 5\omega t + \cdots \right)$$

These odd-order harmonics of the dead-time voltage error interact with the PWM switching
to produce **characteristic current harmonics** at orders 5, 7, 11, 13, 17, 19, … (i.e.,
$$6k \pm 1$$ for $$k = 1,2,3,\ldots$$ in a balanced 3-phase system). These are a well-known
**fingerprint** of inverter dead-time non-linearity, observable in the motor current FFT
spectrum.

> **Important:** The sign-function square wave in the abc-frame naturally contains
> odd harmonics (5th, 7th, 11th, 13th, …). When transformed to the synchronously
> rotating dq-frame, the 5th harmonic (negative-sequence) and 7th harmonic
> (positive-sequence) both project to a **6th-order perturbation** (6ω). This is
> the classic $$6k \pm 1$$ fingerprint seen in motor current spectra — the 6-pulse
> dq-frame pattern is the **result of** abc-frame harmonics, not their cause.

---

## 2. Mathematical Formulation

### 2.1 Per-Sample Correction

The `DeadTimePWM` decorator applies the correction at **every time sample** of the PWM
voltage waveform:

```
For each sample k:
    θ[k] = ω · t[k]                              // Electrical angle

    ia[k] = Id · cos(θ[k]) − Iq · sin(θ[k])      // Phase U current (dq→abc)
    ib[k] = Id · cos(θ[k]−2π/3) − Iq · sin(θ[k]−2π/3)
    ic[k] = Id · cos(θ[k]−4π/3) − Iq · sin(θ[k]−4π/3)

    vu_corrected[k] = vu_nominal[k] − sign(ia[k]) · (td/Ts) · Vdc
    vv_corrected[k] = vv_nominal[k] − sign(ib[k]) · (td/Ts) · Vdc
    vw_corrected[k] = vw_nominal[k] − sign(ic[k]) · (td/Ts) · Vdc
```

Where:
- $$I_d, I_q$$ = steady-state dq currents from one-pass solve (see §4)
- $$\omega = 2\pi \cdot f_{fundamental}$$
- $$t_d$$ = dead-time in seconds (GUI input in µs, converted internally)
- $$T_s = 1 / f_{sw}$$ = switching period
- $$V_{dc}$$ = DC-link voltage (extracted from inner PWM chain or default 400 V)

### 2.2 sign() at Zero-Crossing

C#'s [`Math.Sign`](https://learn.microsoft.com/en-us/dotnet/api/system.math.sign) returns
0 for a zero argument. This means at the precise sample where the current crosses zero
(an infinitesimally narrow window), no correction is applied. This is **physically
plausible**: at zero current there is no freewheeling diode conduction, and the output
is in a high-impedance state. The zero-crossing window is narrow enough (1 sample out
of ~40 per switching period at 200× sub-sampling) to have negligible impact on the
overall harmonic signature.

### 2.3 Magnitude Estimate

For typical values:
- $$t_d = 2\ \mu\text{s}$$, $$f_{sw} = 8\ \text{kHz}$$ → $$T_s = 125\ \mu\text{s}$$
- $$\frac{t_d}{T_s} = \frac{2}{125} = 0.016 = 1.6\%$$
- $$t_d/T_s \cdot V_{dc} = 0.016 \cdot 400\text{ V} = 6.4\text{ V}$$

This means **6.4 V** of per-phase voltage error. On a 230 V phase-to-neutral system, this
is ~2.8% — small but significant enough to produce measurable 5th/7th harmonic currents.

For SiC MOSFETs with $$t_d = 0.5\ \mu\text{s}$$ at 16 kHz: $$0.5/62.5 \cdot 400 = 3.2\text{ V}$$.

---

## 3. Architecture & Integration

### 3.1 Decorator Pattern

`DeadTimePWM` implements the [`ICanOutputVoltage`](PMSMDriveCalc/SPWM.cs:23)
interface, following the same Decorator pattern as [`DCFilteredPWM`](PMSMDriveCalc/GridFilter.cs:127)
and [`OutputLCFilter`](PMSMDriveCalc/LCFilter.cs:13):

```
ICanOutputVoltage
├── SPWM2              (concrete PWM)
├── IPSPWM3            (concrete PWM)
├── QuasiSVPWM2        (concrete PWM)
├── QuasiSVPWM3        (concrete PWM)
├── SVPWM2             (concrete PWM)
├── SVPWM3             (concrete PWM)
├── DeadTimePWM        (decorator — applies dead-time error)
├── DCFilteredPWM      (decorator — injects DC-link ripple)
└── OutputLCFilter     (decorator — applies LC output filter)
```

### 3.2 Class Structure

```csharp
public class DeadTimePWM : ICanOutputVoltage
{
    private readonly ICanOutputVoltage _innerPWM;  // Wrapped PWM strategy
    private readonly PMSMdq _motor;                 // Motor model for current estimation
    private readonly double _deadTime;              // Dead-time in seconds
    private readonly double _vdc;                   // DC-link voltage (extracted from chain)

    public double SwitchingFrequency { get; }       // Delegated to innermost PWM

    public List<List<double>> GetOutputVoltage(
        double amplitudeVoltage,
        double frequencyVoltage,
        double phaseVoltage,
        int periods);
}
```

### 3.3 Decorator Stacking Order

The decorators form a **single serial chain**, each wrapping the accumulated output
of all previous stages:

```
      ┌──────────────┐
      │  Raw PWM     │  (e.g., QuasiSVPWM2)
      │  Strategy    │
      └──────┬───────┘
             │  ideal switching voltages  → finalOutput
      ┌──────▼───────┐
      │ DeadTimePWM  │  ← 1st: dead-time distorts the PWM switching waveform
      └──────┬───────┘     before any other filtering
             │  dead-time-corrected voltages  → finalOutput
      ┌──────▼───────┐
      │ DCFilteredPWM│  ← 2nd: DC-link ripple modulates the dead-time-corrected
      └──────┬───────┘     PWM amplitude → finalOutput
             │
      ┌──────▼───────┐
      │OutputLCFilter│  ← 3rd: LC filter smooths the combined waveform
      └──────┬───────┘     → finalOutput
             │
      ┌──────▼───────┐
      │  Transient   │  ← 4th: solver computes motor currents from final voltage
      │   Solver     │
      └──────────────┘
```

**Rationale:** Dead-time is an **inverter-internal** phenomenon — it modifies the voltage
that the switches actually produce, before any external filtering or DC-link ripple.
Therefore, it must be the **first decorator** in the chain. **Critically**, every
subsequent decorator must wrap `finalOutput` (the accumulated chain), never the raw
`pwm`. This single-accumulator pattern is enforced in both the C# GUI code
([`MainViewModel.cs`](PMSMDriveCalc.UI/ViewModels/MainViewModel.cs:275-302))
and the Python API ([`pmsm_drive_calc.py`](python_scripts/pmsm_drive_calc.py:485-502)).

---

## 4. Steady-State Current Approximation

### 4.1 The Circular Dependency Problem

The dead-time voltage error depends on $$\operatorname{sign}(i_{phase})$$, but
$$i_{phase}$$ is the **output** of the transient solver — which runs **after** the PWM
voltage is generated. This creates a circular dependency:

```
PWM voltage → transient solver → currents → dead-time correction → PWM voltage
     ↑_______________________________________________________________|
```

An exact solution would require an iterative loop (re-run the entire simulation multiple
times), which is computationally expensive (each run computes 100k+ time samples and FFTs).

### 4.2 One-Pass Steady-State Solution

The implementation uses a **one-pass approximation**: the motor current is estimated
from the steady-state dq equations **before** the transient solver runs.

The steady-state dq voltage equations are:

```
Ud = Rs·Id − ω·Lq·Iq
Uq = Rs·Iq + ω·Ld·Id + ω·ψPM
```

Solving for $$I_d, I_q$$ given $$U_d, U_q$$ ([`PMSMdq.SolveSteadyStateCurrents`](PMSMDriveCalc/PMSM.cs:139)):

```csharp
// Matrix form:  [ R   −ωLq ] [Id]   [     Ud      ]
//               [ ωLd   R   ] [Iq] = [ Uq − ω·ψPM ]
//
// Cramer's rule:
det = R² + ω²·Ld·Lq
Id = (R·Ud + ω·Lq·(Uq − ω·ψPM)) / det
Iq = (R·(Uq − ω·ψPM) − ω·Ld·Ud) / det
```

These $$I_d, I_q$$ are then used to reconstruct the 3-phase currents at each time sample
via the inverse Park transform:

```
ia[k] = Id·cos(θ[k]) − Iq·sin(θ[k])
ib[k] = Id·cos(θ[k] − 2π/3) − Iq·sin(θ[k] − 2π/3)
ic[k] = Id·cos(θ[k] − 4π/3) − Iq·sin(θ[k] − 4π/3)
```

### 4.3 Why This is Sufficient

1. **Dead-time correction is small** (~1–3% of DC-link voltage). The feedback effect of
   dead-time on the fundamental current ($$I_d, I_q$$) is second-order and typically
   negligible for harmonic analysis.

2. **Harmonic signature depends on current polarity, not magnitude.** The sign function
   `sign(i_phase)` is insensitive to the exact current amplitude — it only needs the
   correct zero-crossing instants. The steady-state fundamental current provides
   accurate zero-crossing timing.

3. **The 5th/7th harmonic currents caused by dead-time** do feed back into the
   transient solver and affect the final motor current. This is the effect we want to
   observe — the one-pass approximation only affects the dead-time correction itself,
   not the final solved currents.

### 4.4 Speed Handling

The motor's `SpeedRPM` property is temporarily set to match the excitation frequency
before calling `SolveSteadyStateCurrents()`, then restored:

```csharp
double savedRpm = _motor.SpeedRPM;
double rpmFromFreq = frequencyVoltage * 60.0 / (_motor.Poles / 2.0);
_motor.SpeedRPM = rpmFromFreq;
(Id, Iq) = _motor.SolveSteadyStateCurrents(Ud, Uq);
_motor.SpeedRPM = savedRpm;
```

This ensures the correct $$\omega$$ is used in the steady-state equations, even if the
GUI's motor speed setting differs from the excitation frequency (e.g., during
field-weakening studies).

> **Thread safety note:** `GetOutputVoltage()` is called from a background thread
> (via `Task.Run` in the GUI). The temporary mutation and restoration of `SpeedRPM`
> is safe because `PMSMdq.SpeedRPM` is a simple property setter with no side effects,
> and the save/restore sequence runs within a single synchronous call — no other
> thread can observe the intermediate state.

---

## 5. Implementation Details

### 5.1 C# Core: [`DeadTimePWM.cs`](PMSMDriveCalc/DeadTimePWM.cs)

| Aspect | Implementation |
|--------|---------------|
| **Interface** | `ICanOutputVoltage` — compatible with all existing PWM strategies and decorators |
| **Constructor** | `DeadTimePWM(ICanOutputVoltage innerPWM, PMSMdq motor, double deadTimeUs = 2.0)` |
| **Vdc extraction** | If `innerPWM is PWM pwmBase`, uses `pwmBase.DCLink`; otherwise falls back to 400 V |
| **SwitchingFrequency** | Delegates to innermost PWM via `(innerPWM as PWM)?.SwitchingFrequency ?? 8000` |
| **3-phase output** | Handled directly when `nominal.Count >= 4` (QuasiSVPWM, SVPWM, etc.) |
| **Single-phase output** | For SPWM2/IPSPWM3: generates all 3 phases separately, then applies correction |
| **Early exit** | If `dV < 1e-9` (dead-time too small relative to switching period), returns nominal output unchanged |

### 5.1.1 Voltage Reference Convention

The `GetOutputVoltage` interface uses a **sine convention** for the reference voltage:

```
v_ref(t) = amplitudeVoltage · sin(ωt + phaseVoltage)
```

The dq-frame uses a **cosine convention** ($$U_d$$ aligns with the rotor d-axis). The
conversion is:

```
φ_v = phaseVoltage − π/2    (sine → cosine)
Ud = amplitudeVoltage · cos(φ_v)
Uq = amplitudeVoltage · sin(φ_v)
```

This 90° phase shift is handled internally — callers use the sine convention consistently.

### 5.2 GUI Integration: [`MainViewModel.cs`](PMSMDriveCalc.UI/ViewModels/MainViewModel.cs:275-279)

Two properties control the dead-time effect:

```csharp
[ObservableProperty] private bool _enableDeadTimeEffect;  // Checkbox toggle
[ObservableProperty] private double _deadTimeUs = 2.0;    // 0–10 µs, 0.5 µs step
```

In `ComputeAll()`, decorators are stacked in a **single serial chain** — each
decorator wraps the accumulated `finalOutput` from the previous step:

```csharp
ICanOutputVoltage finalOutput = pwm;  // raw PWM strategy

// Step 1: Dead-time voltage error (applied per-sample before all other effects)
if (EnableDeadTimeEffect)
    finalOutput = new DeadTimePWM(finalOutput, motor, DeadTimeUs);

// Step 2: DC-link ripple modulates the dead-time-corrected PWM
if (EnableGridFilter)
    finalOutput = new DCFilteredPWM(finalOutput, grid, DCLink);

// Step 3: LC output filter smooths the final waveform before the motor
if (EnableOutputLCFilter)
    finalOutput = new OutputLCFilter(finalOutput, ...);
```

> **Critical design rule:** Every decorator must wrap `finalOutput` — the accumulated
> chain from all previous steps. Wrapping the raw `pwm` would **discard** all prior
> decorators' voltage corrections. This single serial chain ensures dead-time error,
> DC-link ripple, and LC filtering compose correctly: the dead-time-distorted voltage
> is modulated by DC-link ripple, and that combined waveform is then filtered by
> the LC stage.

The status message includes dead-time information:

```csharp
StatusMessage = $"Complete. ..." +
    (EnableDeadTimeEffect ? $" (dead-time {DeadTimeUs:F1}µs)" : "");
```

### 5.3 GUI: [`MainWindow.axaml`](PMSMDriveCalc.UI/Views/MainWindow.axaml:130-135)

A card in the **Inverter** tab (after PWM Settings, before Filter tabs):

```xml
<CheckBox Content="Enable Dead-Time Effect"
          IsChecked="{Binding EnableDeadTimeEffect}"
          ToolTip.Tip="When enabled, applies a per-sample voltage error
                       ΔV_err = −sign(i_phase) · (td/Ts) · Vdc ..."/>

<NumericUpDown Value="{Binding DeadTimeUs}"
               Minimum="0" Maximum="10" Increment="0.5"
               FormatString="F1"
               IsVisible="{Binding EnableDeadTimeEffect}"/>
```

### 5.4 Python API: [`pmsm_drive_calc.py`](python_scripts/pmsm_drive_calc.py:440-445)

The `PMSMDriveCalcPython.run()` method accepts `dead_time_us: float = 0.0`:

```python
result: ComputationResult = calc.run(
    Vll=400, phase_deg=151.5, fsw=8000, vdc=400,
    mod="SVPWM2", solver="Transient", periods=20,
    dead_time_us=2.0,  # 2 µs dead-time
)
```

When `dead_time_us > 0`, the decorator is wired before grid/LC filters:

```python
if dead_time_us > 0:
    final_output = DeadTimePWM(final_output, motor, dead_time_us)
```

### 5.5 Python API: [`pmsm_tools.py`](python_scripts/pmsm_tools.py)

Three tools expose `dead_time_us`:

| Tool | Purpose |
|------|---------|
| `compute_pmsm_drive` | Full simulation with summary |
| `get_waveforms` | Time-domain waveforms (down-sampled) |
| `get_fft_spectra` | FFT harmonic spectra |

Each tool:
- Accepts `dead_time_us: float = 0.0` in its function signature
- Passes it through to `calc.run(dead_time_us=dead_time_us)`
- Includes `dead_time_us` in its JSON schema under `TOOL_DEFINITIONS`

CLI support via `--dead-time-us`:

```bash
python pmsm_tools.py compute_pmsm_drive --dead-time-us 2.0 ...
```

---

## 6. Expected Harmonic Signatures

### 6.1 Current Harmonics

With dead-time enabled (e.g., $$t_d = 2\ \mu\text{s}$$ at 8 kHz), the motor current FFT
should show elevated amplitudes at:

| Harmonic Order | Source | Relative Amplitude |
|---------------|--------|-------------------|
| 1 (fundamental) | Main excitation | 100% (reference) |
| 5 | Dead-time + PWM | ~1–3% of fundamental |
| 7 | Dead-time + PWM | ~1–3% of fundamental |
| 11 | Dead-time secondary | ~0.5–1% |
| 13 | Dead-time secondary | ~0.5–1% |
| 17, 19 | Higher-order dead-time | <0.5% |

The 5th harmonic is a **negative-sequence** component (rotates opposite to fundamental);
the 7th harmonic is a **positive-sequence** component (rotates with fundamental).

### 6.2 Voltage Harmonics

The dead-time voltage error appears directly in the PWM phase voltage spectrum as a
square-wave-like distortion at the fundamental frequency, producing odd-order
harmonics of the fundamental (3rd, 5th, 7th, 9th, …). In a 3-phase balanced system,
triplen harmonics (3rd, 9th, 15th, …) are common-mode and do not produce line-to-line
voltage or motor current.

---

## 7. Stacking Order & Decorator Chain

### 7.1 Complete Decorator Chain

The full decorator chain is a **single serial pipeline** — each stage wraps the
output of the previous stage:

```
QuasiSVPWM2(fsw, vdc)                              // 0. Concrete PWM strategy
  → DeadTimePWM(output_0, motor, deadTimeUs)       // 1. Dead-time voltage error
  → DCFilteredPWM(output_1, grid, vdc)             // 2. DC-link ripple modulation
  → OutputLCFilter(output_2, Lf, Cf, ...)          // 3. LC output filter
  → PMSMDQDriveCalculator.Compute(Ud, Uq, ...)     // 4. Transient solve + FFT
```

Every `output_N` is the accumulated chain result up to stage N. This serial
architecture guarantees that downstream stages see the corrected voltages from
all upstream stages.

In code (C#):

```csharp
ICanOutputVoltage finalOutput = pwm;
if (EnableDeadTimeEffect)
    finalOutput = new DeadTimePWM(finalOutput, motor, DeadTimeUs);
if (EnableGridFilter)
    finalOutput = new DCFilteredPWM(finalOutput, grid, DCLink);
if (EnableOutputLCFilter)
    finalOutput = new OutputLCFilter(finalOutput, ...);
```

### 7.2 Interaction with Other Decorators

| Decorator | Interaction with DeadTimePWM |
|-----------|------------------------------|
| **DCFilteredPWM** | Receives the dead-time-corrected voltage. DC-link ripple amplitude modulation is applied to the already-distorted PWM waveform — this is physically correct: the inverter switches the dead-time-affected voltage against the actual (rippling) DC-link. |
| **OutputLCFilter** | Receives the combined (dead-time + DC-ripple) voltage. The LC filter attenuates high-frequency harmonics (including switching-frequency components) but passes the low-order 5th/7th harmonics, so the dead-time signature remains clearly visible in motor current. |

### 7.3 Design Decision: Serial Chain (Single Accumulator)

All decorators share one `finalOutput` accumulator — each wraps the output of the
previous stage:

```csharp
ICanOutputVoltage finalOutput = pwm;                                // 0. Raw PWM
finalOutput = new DeadTimePWM(finalOutput, motor, DeadTimeUs);      // 1. + Dead-time
finalOutput = new DCFilteredPWM(finalOutput, grid, DCLink);         // 2. + DC Ripple
finalOutput = new OutputLCFilter(finalOutput, ...);                 // 3. + LC Filter
```

This serial ordering reflects the physical signal path in a real drive:

1. The inverter inserts dead-time into the PWM switching waveform (gate-driver level).
2. The resulting voltage is then subject to DC-link voltage ripple (bus level).
3. The combined waveform passes through the output LC filter before reaching the motor.

For harmonic analysis, both dead-time and DC-link ripple are small perturbations
(<3% each). Their cross-coupling (dead-time error magnitude varying with DC-link
voltage ripple) is second-order but **is captured** by the serial chain architecture.

---

## 8. Usage Examples

### 8.1 C# GUI

1. Open the **Inverter** tab
2. Check **"Enable Dead-Time Effect"**
3. Set **Dead-Time (µs)** to `2.0` (typical IGBT) or `0.5` (SiC MOSFET)
4. Click **Compute**
5. Inspect the **Current FFT** plot — look for elevated 5th and 7th harmonics
6. Toggle dead-time on/off and compare FFT spectra

### 8.2 C# Programmatic

```csharp
var motor = new PMSMdq(poles: 8, psiPM: 0.125, Ld: 0.0025, Lq: 0.005, R: 0.15);
motor.SpeedRPM = 3000;

var basePwm = new QuasiSVPWM2(fsw: 8000, vdc: 400);
var pwm = new DeadTimePWM(basePwm, motor, deadTimeUs: 2.0);

var calc = new PMSMDQDriveCalculator(motor, pwm);

// Calculate Ud, Uq from Vll and phase angle
double Vpn = 400.0 / Math.Sqrt(3);
double phi = (151.5 - 90) * Math.PI / 180.0;
double Ud = Vpn * Math.Cos(phi);
double Uq = Vpn * Math.Sin(phi);

var result = calc.Compute(Ud, Uq, SolverType.Transient, periods: 20);

// Result.IU_FFT now contains the current spectrum with dead-time harmonics
Console.WriteLine($"5th harmonic: {result.IU_FFT.Amplitudes[4]}");
Console.WriteLine($"7th harmonic: {result.IU_FFT.Amplitudes[6]}");
```

### 8.3 Python API

```python
from pmsm_drive_calc import PMSMDriveCalcPython

calc = PMSMDriveCalcPython(
    poles=8, R=0.15, Ld=0.0025, Lq=0.005, psi_pm=0.125, speed=3000
)

# Without dead-time (baseline)
result_ideal = calc.run(Vll=400, phase_deg=151.5)

# With 2 µs dead-time
result_dt = calc.run(Vll=400, phase_deg=151.5, dead_time_us=2.0)

# Compare 5th harmonic current
print(f"Without DT: 5th = {result_ideal.IU_FFT.amplitudes[4]:.4f} A")
print(f"With DT:    5th = {result_dt.IU_FFT.amplitudes[4]:.4f} A")
```

### 8.4 Python LLM Tool Interface

```python
from pmsm_tools import run_tool

result = run_tool("compute_pmsm_drive", {
    "poles": 8, "speed": 3000, "Vll": 400, "phase_deg": 151.5,
    "dead_time_us": 2.0
})
print(result["fft_summary"])
```

---

## 9. Limitations & Assumptions

### 9.1 Assumptions

| Assumption | Justification |
|-----------|---------------|
| **Sinusoidal steady-state currents** | The sign function only needs zero-crossing timing, which is well-approximated by the fundamental component of the current. |
| **No iterative feedback** | Dead-time correction is ~1–3% of Vdc. Its effect on Id/Iq (fundamental) is second-order and negligible for harmonic analysis. |
| **Ideal switches** | No voltage drop across IGBT/MOSFET (Vce_sat, Rds_on) or diode (Vf). These add a constant offset that can be absorbed into the effective dead-time. |
| **Constant DC-link voltage** | Vdc is treated as constant during dead-time. Ripple effects are handled separately by `DCFilteredPWM`. |
| **Balanced 3-phase system** | All three legs have identical dead-time. In practice, there is always some mismatch (±10–20 ns), which produces additional even-order harmonics. |

### 9.2 Known Limitations

1. **No iterative convergence:** The one-pass steady-state approximation does not account
   for the dead-time effect's influence on the fundamental Id/Iq operating point. For
   high-power drives with large dead-time (>4 µs), the fundamental voltage drop may
   become significant (>5%), and an iterative re-solve would improve accuracy.

2. **No dead-time compensation:** The model only adds the dead-time distortion — it does
   not implement any compensation strategy (e.g., predictive current control, disturbance
   observer). This is by design: the purpose is to analyze the effect, not counteract it.

3. **Per-sample correction vs. per-switching-period average:** The correction is applied
   at every sample (200× per switching period), while the physical dead-time effect is
   a per-switching-period average. Since the sign function only changes near current
   zero-crossings (a narrow window), the per-sample approach accurately captures the
   average behavior.

4. **Switching-frequency sidebands are captured:** Because the dead-time correction
   is applied at **every time sample** (200× per switching period), not as a per-period
   average, it naturally captures the interaction between dead-time voltage error and
   switching-frequency ripple. The corrected waveform at each sample includes both the
   fundamental-frequency dead-time distortion and the instantaneous switching-state
   effects. This means switching-frequency sideband harmonics (e.g., $$f_{sw} \pm 6f_0$$)
   are present in the output FFT — an advantage of the per-sample approach over simpler
   average-value models.

5. **No temperature dependence:** Switching device turn-on/turn-off times vary with
   junction temperature. The effective dead-time may differ from the programmed value
   by 10–20%.

### 9.3 When to Use / When Not to Use

| Scenario | Recommendation |
|----------|---------------|
| Harmonic analysis of motor current (5th, 7th, …) | ✅ Enable dead-time |
| Comparing modulation strategies at the fundamental | ❌ Keep disabled (dead-time is inverter-dependent, not modulation-dependent) |
| LC filter design (resonance analysis) | ✅ Enable for realistic excitation |
| Efficiency / loss estimation | ⚠️ Useful for qualitative comparison, not quantitative accuracy |
| Controller bandwidth / stability analysis | ❌ Out of scope — use a real-time simulator |

---

## 10. GUI & Python API Surface

### 10.1 GUI Controls

| Control | Property Binding | Range | Default | Description |
|---------|-----------------|-------|---------|-------------|
| CheckBox | `EnableDeadTimeEffect` | on/off | `false` | Toggle dead-time effect on/off |
| NumericUpDown | `DeadTimeUs` | 0 – 10 µs, step 0.5 | `2.0` | Dead-time duration in microseconds |

### 10.2 Python API Parameters

| Module | Parameter | Type | Default | Description |
|--------|-----------|------|---------|-------------|
| `pmsm_drive_calc.run()` | `dead_time_us` | `float` | `0.0` | Dead-time in µs (0 = disabled) |
| `pmsm_tools.compute_pmsm_drive()` | `dead_time_us` | `float` | `0.0` | Same |
| `pmsm_tools.get_waveforms()` | `dead_time_us` | `float` | `0.0` | Same |
| `pmsm_tools.get_fft_spectra()` | `dead_time_us` | `float` | `0.0` | Same |

### 10.3 CLI Flags

```
--dead-time-us FLOAT    Dead-time in microseconds (default: 0.0)
```

### 10.4 Status Message Format

The GUI status bar appends dead-time information when enabled:

```
Complete. 40001 time points, Torque=12.3456 Nm (dead-time 2.0µs)
```

---

## References

1. Murai, Y., et al. "Waveform distortion and correction circuit for PWM inverters
   with switching lag-times." *IEEE Trans. Industry Applications*, 1987.
2. Holtz, J. "Pulsewidth modulation for electronic power conversion." *Proceedings of
   the IEEE*, 1994.
3. Jeong, S.G., Park, M.H. "The analysis and compensation of dead-time effects in PWM
   inverters." *IEEE Trans. Industrial Electronics*, 1991.
4. [`PMSMDriveCalc`](https://github.com) — Source code, this project.
