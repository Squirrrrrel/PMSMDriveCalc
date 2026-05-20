#!/usr/bin/env python3
"""
PMSM Drive Calculator — LLM Tool Interface
============================================

Exposes the PMSM drive simulation as standalone, callable functions suitable for
LLM tool-calling (OpenAI / Anthropic / Gemini function-calling format).

Design principles:
- Every function returns a plain Python dict (directly JSON-serialisable).
- No mutable global state — each call is self-contained.
- Full type hints and exhaustive docstrings for LLM comprehension.
- Includes pre-built tool definitions in OpenAI function-calling schema format.

Usage as a Python module:
    from pmsm_tools import compute_pmsm_drive, get_tool_definitions, run_tool

    # Direct call
    result = compute_pmsm_drive(poles=8, speed=3000, Vll=400, phase_deg=151.5)

    # Via tool dispatcher (suitable for LLM agents)
    tools = get_tool_definitions()          # list of OpenAI-format tool schemas
    reply = run_tool("compute_pmsm_drive", {
        "poles": 8, "speed": 3000,
        "Vll": 400, "phase_deg": 151.5,
        "mod": "SVPWM2", "solver": "Transient",
    })

Usage from CLI:
    python pmsm_tools.py --poles 8 --speed 3000 --Vll 400 --phase 151.5
"""

from __future__ import annotations
import argparse
import json
import os
import sys
from dataclasses import asdict
from typing import Any, Dict, List, Optional

# ---------------------------------------------------------------------------
# Bootstrap: ensure pmsm_drive_calc.py is importable from the same directory
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from pmsm_drive_calc import (
    PMSMDriveCalcPython,
    ComputationResult,
    OperatingPoint,
    FFTData,
)

# ---------------------------------------------------------------------------
# Tool: compute_pmsm_drive
# ---------------------------------------------------------------------------

def compute_pmsm_drive(
    # ── Motor ──
    poles: int = 8,
    R: float = 0.15,
    Ld: float = 0.0025,
    Lq: float = 0.005,
    psi_pm: float = 0.125,
    speed: float = 3000.0,
    L_sigma_ratio: float = 0.1,
    # ── Voltage / Inverter ──
    Vll: float = 400.0,
    phase_deg: float = 151.5,
    fsw: float = 8000.0,
    vdc: float = 400.0,
    mod: str = "SVPWM2",
    third_harmonic: bool = False,
    # ── Solver ──
    solver: str = "Transient",
    periods: int = 20,
    # ── LC output filter ──
    lc_filter: bool = False,
    Lf: float = 0.0004,
    Cf: float = 0.000025,
    # ── Grid DC-link ripple filter ──
    grid_filter: bool = False,
    grid_Vpk: float = 245.0,
    grid_freq: float = 50.0,
    dc_cap: float = 0.0035,
    dead_time_us: float = 0.0,
    dc_ind: float = 0.0005,
) -> Dict[str, Any]:
    """
    Run the full PMSM drive simulation.

    Parameters
    ----------
    poles : int
        Number of rotor poles (must be even). Electrical frequency = speed/60 × poles/2.
    R : float
        Stator phase resistance in ohms.
    Ld : float
        d-axis inductance in henries.
    Lq : float
        q-axis inductance in henries. Lq > Ld for salient-pole IPM motors.
    psi_pm : float
        Permanent-magnet flux linkage in webers.
    speed : float
        Rotor mechanical speed in RPM.
    L_sigma_ratio : float
        Leakage/d-axis inductance ratio (Lσ/Ld) (default 0.1, clamped 0.0–0.5).
        Used for LC filter high-frequency impedance modelling.
    Vll : float
        Target line-to-line peak voltage in volts. Internally converted to Ud/Uq
        via Vpn_peak = Vll / sqrt(3).
    phase_deg : float
        Voltage phasor angle in the dq-frame in degrees (atan2(Uq, Ud)).
        151.5° gives Ud ≈ -203V, Uq ≈ +110V at Vll=400 (typical MTPA region).
    fsw : float
        PWM switching frequency in Hz.
    vdc : float
        DC-link voltage in volts. Sets the maximum fundamental output:
        vdc for SVPWM, vdc × √3/2 for SPWM.
    mod : str
        Modulation type. One of: 'SPWM2', 'IPSPWM3', 'SVPWM2', 'SVPWM3',
        'QuasiSVPWM2', 'QuasiSVPWM3'.
        SVPWM2 is the default. Note: 'SVPWM2' and 'SVPWM3' are internally
        implemented via QuasiSVPWM (zero-sequence injection/saddle waveform),
        mathematically equivalent to traditional SVPWM but without ZOH phase lag.
    third_harmonic : bool
        Enable 1/6 third-harmonic injection. Only valid with SPWM2 or IPSPWM3.
        Increases linear modulation range by ~15%.
    solver : str
        Solver type. One of: 'AC' (redirected to Transient), 'Transient' (DQ 2×2),
        'Transient+LC' (legacy ABC 6-state),
        'Transient+LC-Full' (DQ 6-state fully coupled).
    periods : int
        Number of fundamental electrical periods to simulate. More periods =
        better FFT resolution, longer runtime. Typical: 20.
    lc_filter : bool
        Enable Y-connected LC output filter between inverter and motor terminals.
    Lf : float
        Filter inductance per phase in henries. Typical: 100 µH – 1 mH.
    Cf : float
        Filter capacitance per phase in farads (Y-connected, floating star point).
        Typical: 5–100 µF.
    grid_filter : bool
        Enable grid-side diode-bridge rectifier DC-link ripple simulation.
    grid_Vpk : float
        Grid phase voltage amplitude in volts (~230 Vrms × √2 = 325 V).
    grid_freq : float
        Grid frequency in Hz (50 or 60).
    dc_cap : float
        DC-link capacitance in farads.
    dc_ind : float
        DC-link series inductance in henries.
    dead_time_us : float
        Dead-time (blanking time) in microseconds. When > 0, a DeadTimePWM
        decorator applies per-sample voltage error ΔV_err = −sign(i_phase) ·
        (td/Ts) · Vdc. Typical IGBT: 1–4 µs, SiC MOSFET: 0.2–1 µs.
        Introduces characteristic 5th/7th harmonic signature. Default 0.0.

    Returns
    -------
    dict
        A JSON-serialisable dictionary with keys:
        - 'operating_point': dict of steady-state values (Id, Iq, torque, power, etc.)
        - 'waveform_summary': dict with signal lengths, base frequency, duration
        - 'waveform_preview': first 10 samples of key waveforms for quick inspection
        - 'fft_summary': fundamental and dominant harmonics for IU, VU
        - 'fft_available': list of all FFT channel keys that have data
    """
    calc = PMSMDriveCalcPython(
        poles=poles, R=R, Ld=Ld, Lq=Lq, psi_pm=psi_pm, speed=speed,
        L_sigma_ratio=L_sigma_ratio,
    )
    result: ComputationResult = calc.run(
        Vll=Vll, phase_deg=phase_deg, fsw=fsw, vdc=vdc,
        mod=mod, third_harmonic=third_harmonic,
        solver=solver, periods=periods,
        lc_filter=lc_filter, Lf=Lf, Cf=Cf,
        grid_filter=grid_filter,
        grid_Vpk=grid_Vpk, grid_freq=grid_freq,
        dc_cap=dc_cap, dc_ind=dc_ind,
        dead_time_us=dead_time_us,
    )

    # ── Build a flat, JSON-friendly operating-point dict ──
    op = result.op
    op_dict: Dict[str, Any] = {
        "Id_A": round(op.Id, 6),
        "Iq_A": round(op.Iq, 6),
        "Ud_V": round(op.Ud, 6),
        "Uq_V": round(op.Uq, 6),
        "current_magnitude_A": round(op.CurrentMagnitude, 6),
        "current_angle_deg": round(op.CurrentAngleRad * 180.0 / 3.141592653589793, 4),
        "voltage_magnitude_V": round(op.PhaseVoltageMagnitude, 4),
        "voltage_angle_deg": round(op.VoltageAngleRad * 180.0 / 3.141592653589793, 4),
        "V_LL_peak_V": round(op.PhaseVoltageMagnitude * 1.7320508075688772, 4),
        "torque_Nm": round(op.TorqueNm, 4),
        "mechanical_power_W": round(op.MechanicalPowerW, 2),
        "active_power_W": round(op.ActivePowerW, 2),
        "apparent_power_VA": round(op.ApparentPowerVA, 2),
        "pwm_apparent_power_VA": round(op.PwmApparentPowerVA, 2),
        "power_factor": round(op.PowerFactor, 4),
        "copper_loss_W": round(op.CopperLossW, 2),
        "motor_Ud_V": round(op.MotorUd, 4),
        "motor_Uq_V": round(op.MotorUq, 4),
        "motor_V_dq_mag_V": round(
            (op.MotorUd ** 2 + op.MotorUq ** 2) ** 0.5, 4
        ),
        "speed_RPM": round(op.SpeedRPM, 1),
        "electrical_freq_Hz": round(op.ElectricalFreqHz, 2),
        "poles": op.Poles,
        "phase_resistance_ohm": round(op.PhaseResistance, 4),
        "Ld_H": round(op.Ld, 6),
        "Lq_H": round(op.Lq, 6),
        "psi_pm_Wb": round(op.PMFluxLinkage, 4),
    }

    # ── Waveform summary ──
    n = len(result.time)
    waveform_summary: Dict[str, Any] = {
        "num_samples": n,
        "duration_s": round(result.time[-1] if n else 0.0, 6),
        "base_frequency_Hz": round(result.BaseFrequencyHz, 2),
        "PWM_V_LL_fundamental_peak_V": round(result.PwmVPhasePhaseFundamental, 4),
    }
    if result.MotorVPhaseNeutralRms is not None:
        waveform_summary["motor_V_PN_RMS_V"] = round(result.MotorVPhaseNeutralRms, 4)
    if result.MotorVPhasePhaseFundamental is not None:
        waveform_summary["motor_V_LL_fundamental_peak_V"] = round(
            result.MotorVPhasePhaseFundamental, 4
        )

    # ── Waveform preview (first 10 samples) ──
    preview_count = min(10, n)
    waveform_preview: Dict[str, List[float]] = {}
    for key in ["time", "VU", "VV", "VW", "VUV", "VVW", "VWU", "IU", "IV", "IW"]:
        arr = getattr(result, key, None)
        if arr:
            waveform_preview[key] = [round(arr[i], 6) for i in range(preview_count)]
    # Motor waveforms (LC filter)
    for key in ["MotorVU", "MotorVV", "MotorVW", "MotorVUV", "MotorVVW", "MotorVWU"]:
        arr = getattr(result, key, None)
        if arr:
            waveform_preview[key] = [round(arr[i], 6) for i in range(preview_count)]

    # ── FFT summary ──
    fft_summary: Dict[str, Any] = {}
    fft_available: List[str] = []

    def _summarise_fft(label: str, fft: Optional[FFTData]) -> Optional[Dict[str, Any]]:
        if fft is None or not fft.orders:
            return None
        # Fundamental (order 1)
        fund_idx = next((i for i, o in enumerate(fft.orders) if o == 1), None)
        fundamental = (
            round(fft.amplitudes[fund_idx], 6) if fund_idx is not None else 0.0
        )
        # Dominant harmonics (top 5 by amplitude, excluding order 1)
        indexed = [
            (o, a, p)
            for o, a, p in zip(fft.orders, fft.amplitudes, fft.phases_deg)
            if o > 1
        ]
        indexed.sort(key=lambda x: x[1], reverse=True)
        top5 = [
            {"order": o, "amplitude": round(a, 6), "phase_deg": round(p, 4)}
            for o, a, p in indexed[:5]
        ]
        thd = (
            round((sum(x[1] ** 2 for x in indexed) ** 0.5 / fundamental) * 100, 4)
            if fundamental > 0
            else 0.0
        )
        return {
            "fundamental_order_1": fundamental,
            "THD_percent": thd,
            "top_harmonics": top5,
        }

    for label, attr in [
        ("IU", "IU_FFT"), ("IV", "IV_FFT"), ("IW", "IW_FFT"),
        ("VU", "VU_FFT"), ("VV", "VV_FFT"), ("VW", "VW_FFT"),
        ("VUV", "VUV_FFT"), ("VVW", "VVW_FFT"), ("VWU", "VWU_FFT"),
        ("MotorVU", "MotorVU_FFT"), ("MotorVV", "MotorVV_FFT"),
        ("MotorVW", "MotorVW_FFT"),
        ("MotorVUV", "MotorVUV_FFT"), ("MotorVVW", "MotorVVW_FFT"),
        ("MotorVWU", "MotorVWU_FFT"),
    ]:
        fft = getattr(result, attr, None)
        s = _summarise_fft(label, fft)
        if s is not None:
            fft_summary[label] = s
            fft_available.append(label)

    return {
        "operating_point": op_dict,
        "waveform_summary": waveform_summary,
        "waveform_preview": waveform_preview,
        "fft_summary": fft_summary,
        "fft_available": fft_available,
    }


# ---------------------------------------------------------------------------
# Tool: get_waveforms
# ---------------------------------------------------------------------------

def get_waveforms(
    poles: int = 8,
    R: float = 0.15,
    Ld: float = 0.0025,
    Lq: float = 0.005,
    psi_pm: float = 0.125,
    speed: float = 3000.0,
    L_sigma_ratio: float = 0.1,
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
    dead_time_us: float = 0.0,
    max_samples: int = 500,
) -> Dict[str, Any]:
    """
    Run the PMSM drive simulation and return the time-domain waveforms.

    This is the same computation as compute_pmsm_drive but returns the full
    waveform arrays (down-sampled to max_samples for LLM context limits).

    Parameters
    ----------
    dead_time_us : float
        Dead-time (blanking time) in microseconds. When > 0, a DeadTimePWM
        decorator applies per-sample voltage error to simulate inverter
        non-linearity (default 0.0 = disabled).
    max_samples : int
        Maximum number of time samples to return (default 500). The raw arrays
        can have 100K+ samples; this down-samples for LLM-friendly output.

    All other parameters: identical to compute_pmsm_drive().

    Returns
    -------
    dict
        Keys: 'time', 'VU', 'VV', 'VW', 'VUV', 'VVW', 'VWU', 'IU', 'IV', 'IW'
        (plus MotorVU..MotorVWU when lc_filter=True, and 'operating_point').
        All values are lists of floats.
    """
    calc = PMSMDriveCalcPython(
        poles=poles, R=R, Ld=Ld, Lq=Lq, psi_pm=psi_pm, speed=speed,
        L_sigma_ratio=L_sigma_ratio,
    )
    result: ComputationResult = calc.run(
        Vll=Vll, phase_deg=phase_deg, fsw=fsw, vdc=vdc,
        mod=mod, third_harmonic=third_harmonic,
        solver=solver, periods=periods,
        lc_filter=lc_filter, Lf=Lf, Cf=Cf,
        grid_filter=grid_filter,
        grid_Vpk=grid_Vpk, grid_freq=grid_freq,
        dc_cap=dc_cap, dc_ind=dc_ind,
        dead_time_us=dead_time_us,
    )

    n = len(result.time)
    step = max(1, n // max_samples)

    def _downsample(arr: List[float]) -> List[float]:
        return [round(arr[i], 6) for i in range(0, n, step)]

    time_downsampled = _downsample(result.time)
    out: Dict[str, Any] = {
        "time": time_downsampled,
        "VU": _downsample(result.VU),
        "VV": _downsample(result.VV),
        "VW": _downsample(result.VW),
        "VUV": _downsample(result.VUV),
        "VVW": _downsample(result.VVW),
        "VWU": _downsample(result.VWU),
        "IU": _downsample(result.IU),
        "IV": _downsample(result.IV),
        "IW": _downsample(result.IW),
        "operating_point": {
            "Id_A": round(result.op.Id, 6),
            "Iq_A": round(result.op.Iq, 6),
            "torque_Nm": round(result.op.TorqueNm, 4),
            "speed_RPM": round(result.op.SpeedRPM, 1),
            "power_W": round(result.op.MechanicalPowerW, 2),
            "power_factor": round(result.op.PowerFactor, 4),
        },
        "num_samples_returned": len(time_downsampled),
        "num_samples_raw": n,
    }

    # Motor-side waveforms (LC filter)
    if result.MotorVU is not None:
        for key in ["MotorVU", "MotorVV", "MotorVW",
                     "MotorVUV", "MotorVVW", "MotorVWU"]:
            arr = getattr(result, key, None)
            if arr:
                out[key] = _downsample(arr)

    return out


# ---------------------------------------------------------------------------
# Tool: get_fft_spectra
# ---------------------------------------------------------------------------

def get_fft_spectra(
    poles: int = 8,
    R: float = 0.15,
    Ld: float = 0.0025,
    Lq: float = 0.005,
    psi_pm: float = 0.125,
    speed: float = 3000.0,
    L_sigma_ratio: float = 0.1,
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
    dead_time_us: float = 0.0,
    max_harmonics: int = 100,
) -> Dict[str, Any]:
    """
    Run the PMSM drive simulation and return the FFT spectra for all channels.

    Parameters
    ----------
    dead_time_us : float
        Dead-time (blanking time) in microseconds. When > 0, a DeadTimePWM
        decorator applies per-sample voltage error to simulate inverter
        non-linearity (default 0.0 = disabled).
    max_harmonics : int
        Maximum harmonic order to include (default 100). Fundamental = order 1.

    All other parameters: identical to compute_pmsm_drive().

    Returns
    -------
    dict
        Keys for each channel (e.g., 'IU', 'VU', ...). Each value is a dict with:
        - 'orders': list of harmonic orders
        - 'amplitudes': list of amplitudes
        - 'phases_deg': list of phase angles in degrees
        Also includes 'base_frequency_Hz' and 'operating_point'.
    """
    calc = PMSMDriveCalcPython(
        poles=poles, R=R, Ld=Ld, Lq=Lq, psi_pm=psi_pm, speed=speed,
        L_sigma_ratio=L_sigma_ratio,
    )
    result: ComputationResult = calc.run(
        Vll=Vll, phase_deg=phase_deg, fsw=fsw, vdc=vdc,
        mod=mod, third_harmonic=third_harmonic,
        solver=solver, periods=periods,
        lc_filter=lc_filter, Lf=Lf, Cf=Cf,
        grid_filter=grid_filter,
        grid_Vpk=grid_Vpk, grid_freq=grid_freq,
        dc_cap=dc_cap, dc_ind=dc_ind,
        dead_time_us=dead_time_us,
    )

    out: Dict[str, Any] = {
        "base_frequency_Hz": round(result.BaseFrequencyHz, 2),
        "operating_point": {
            "Id_A": round(result.op.Id, 6),
            "Iq_A": round(result.op.Iq, 6),
            "torque_Nm": round(result.op.TorqueNm, 4),
            "speed_RPM": round(result.op.SpeedRPM, 1),
        },
    }

    def _extract_fft(fft: Optional[FFTData]) -> Optional[Dict[str, Any]]:
        if fft is None or not fft.orders:
            return None
        orders, amps, phases = [], [], []
        for o, a, p in zip(fft.orders, fft.amplitudes, fft.phases_deg):
            if o > max_harmonics:
                break
            orders.append(o)
            amps.append(round(a, 6))
            phases.append(round(p, 4))
        return {"orders": orders, "amplitudes": amps, "phases_deg": phases}

    for label, attr in [
        ("IU", "IU_FFT"), ("IV", "IV_FFT"), ("IW", "IW_FFT"),
        ("VU", "VU_FFT"), ("VV", "VV_FFT"), ("VW", "VW_FFT"),
        ("VUV", "VUV_FFT"), ("VVW", "VVW_FFT"), ("VWU", "VWU_FFT"),
        ("MotorVU", "MotorVU_FFT"), ("MotorVV", "MotorVV_FFT"),
        ("MotorVW", "MotorVW_FFT"),
        ("MotorVUV", "MotorVUV_FFT"), ("MotorVVW", "MotorVVW_FFT"),
        ("MotorVWU", "MotorVWU_FFT"),
    ]:
        fft = getattr(result, attr, None)
        s = _extract_fft(fft)
        if s is not None:
            out[label] = s

    return out


# ---------------------------------------------------------------------------
# Tool: compute_operating_point (lightweight: DQ steady-state, no waveforms)
# ---------------------------------------------------------------------------

def compute_operating_point(
    poles: int = 8,
    R: float = 0.15,
    Ld: float = 0.0025,
    Lq: float = 0.005,
    psi_pm: float = 0.125,
    speed: float = 3000.0,
    L_sigma_ratio: float = 0.1,
    Vll: float = 400.0,
    phase_deg: float = 151.5,
) -> Dict[str, Any]:
    """
    Compute only the steady-state operating point using DQ steady-state solve.

    This is much faster than the full transient simulation (~milliseconds)
    because it skips PWM waveform generation and FFT.  Use this when you
    only need Id, Iq, torque, and power values.

    Parameters
    ----------
    poles, R, Ld, Lq, psi_pm, speed, L_sigma_ratio : motor parameters (see compute_pmsm_drive).
    Vll : float
        Target line-to-line peak voltage in volts.
    phase_deg : float
        Voltage phasor angle in the dq-frame in degrees.

    Returns
    -------
    dict
        Steady-state operating point with keys: Id_A, Iq_A, torque_Nm,
        power_W, power_factor, Ud_V, Uq_V, current_magnitude_A,
        current_angle_deg, voltage_angle_deg, speed_RPM, electrical_freq_Hz.
    """
    calc = PMSMDriveCalcPython(
        poles=poles, R=R, Ld=Ld, Lq=Lq, psi_pm=psi_pm, speed=speed,
        L_sigma_ratio=L_sigma_ratio,
    )
    result: ComputationResult = calc.run(
        Vll=Vll, phase_deg=phase_deg,
        mod="SVPWM2", solver="AC", periods=1,
    )
    op = result.op
    return {
        "Id_A": round(op.Id, 6),
        "Iq_A": round(op.Iq, 6),
        "Ud_V": round(op.Ud, 4),
        "Uq_V": round(op.Uq, 4),
        "current_magnitude_A": round(op.CurrentMagnitude, 6),
        "current_angle_deg": round(op.CurrentAngleRad * 180.0 / 3.141592653589793, 4),
        "voltage_magnitude_V": round(op.PhaseVoltageMagnitude, 4),
        "voltage_angle_deg": round(op.VoltageAngleRad * 180.0 / 3.141592653589793, 4),
        "V_LL_peak_V": round(op.PhaseVoltageMagnitude * 1.7320508075688772, 4),
        "torque_Nm": round(op.TorqueNm, 4),
        "mechanical_power_W": round(op.MechanicalPowerW, 2),
        "active_power_W": round(op.ActivePowerW, 2),
        "apparent_power_VA": round(op.ApparentPowerVA, 2),
        "power_factor": round(op.PowerFactor, 4),
        "copper_loss_W": round(op.CopperLossW, 2),
        "speed_RPM": round(op.SpeedRPM, 1),
        "electrical_freq_Hz": round(op.ElectricalFreqHz, 2),
        "poles": op.Poles,
        "phase_resistance_ohm": round(op.PhaseResistance, 4),
        "Ld_H": round(op.Ld, 6),
        "Lq_H": round(op.Lq, 6),
        "psi_pm_Wb": round(op.PMFluxLinkage, 4),
    }


# ---------------------------------------------------------------------------
# Tool definitions (OpenAI function-calling format)
# ---------------------------------------------------------------------------

TOOL_DEFINITIONS: List[Dict[str, Any]] = [
    {
        "type": "function",
        "function": {
            "name": "compute_pmsm_drive",
            "description": (
                "Run a complete PMSM (Permanent Magnet Synchronous Motor) drive "
                "simulation. Computes the steady-state operating point (Id, Iq, "
                "torque, power, power factor, efficiency), time-domain PWM "
                "waveforms, and FFT harmonic spectra. Supports six modulation "
                "strategies (SPWM2, IPSPWM3, SVPWM2, SVPWM3, QuasiSVPWM2, "
                "QuasiSVPWM3), five solvers "
                "(AC, Transient, Transient+LC, Transient+LC-Full), "
                "optional Y-connected LC output filter, and optional grid-side "
                "DC-link ripple filter. ""SVPWM2\"/\"SVPWM3\" are internally "
                "implemented via QuasiSVPWM (zero-sequence injection), "
                "mathematically equivalent to traditional space-vector PWM "
                "but without ZOH phase lag. All quantities are in SI units."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "poles": {
                        "type": "integer",
                        "description": "Number of rotor poles (must be even).",
                        "default": 8,
                    },
                    "R": {
                        "type": "number",
                        "description": "Stator phase resistance (ohms).",
                        "default": 0.15,
                    },
                    "Ld": {
                        "type": "number",
                        "description": "d-axis inductance (henries).",
                        "default": 0.0025,
                    },
                    "Lq": {
                        "type": "number",
                        "description": "q-axis inductance (henries). Lq > Ld for IPM.",
                        "default": 0.005,
                    },
                    "psi_pm": {
                        "type": "number",
                        "description": "PM flux linkage (webers).",
                        "default": 0.125,
                    },
                    "speed": {
                        "type": "number",
                        "description": "Rotor mechanical speed (RPM).",
                        "default": 3000,
                    },
                    "L_sigma_ratio": {
                        "type": "number",
                        "description": (
                            "Leakage inductance ratio Lσ/Ld (default 0.1). "
                            "Used for LC filter high-frequency impedance. "
                            "Range 0.0–0.5."
                        ),
                        "default": 0.1,
                    },
                    "Vll": {
                        "type": "number",
                        "description": (
                            "Target line-to-line peak voltage (volts). "
                            "V_PN_peak = Vll / sqrt(3)."
                        ),
                        "default": 400,
                    },
                    "phase_deg": {
                        "type": "number",
                        "description": (
                            "Voltage phasor angle in dq-frame (degrees). "
                            "151.5° gives typical MTPA operating point."
                        ),
                        "default": 151.5,
                    },
                    "fsw": {
                        "type": "number",
                        "description": "PWM switching frequency (Hz).",
                        "default": 8000,
                    },
                    "vdc": {
                        "type": "number",
                        "description": "DC-link voltage (volts).",
                        "default": 400,
                    },
                    "mod": {
                        "type": "string",
                        "enum": [
                            "SPWM2",
                            "IPSPWM3",
                            "SVPWM2",
                            "SVPWM3",
                            "QuasiSVPWM2",
                            "QuasiSVPWM3",
                        ],
                        "description": (
                            "Modulation strategy. SVPWM2/SVPWM3 are internally "
                            "implemented via QuasiSVPWM (zero-sequence injection), "
                            "which avoids ZOH phase lag while being mathematically "
                            "equivalent to traditional space-vector PWM."
                        ),
                        "default": "SVPWM2",
                    },
                    "third_harmonic": {
                        "type": "boolean",
                        "description": (
                            "Enable 1/6 third-harmonic injection. "
                            "Only valid with SPWM2 or IPSPWM3."
                        ),
                        "default": False,
                    },
                    "solver": {
                        "type": "string",
                        "enum": ["AC", "Transient", "Transient+LC", "Transient+LC-Full"],
                        "description": (
                            "Solver type. AC = redirected to Transient (DQ 2×2). "
                            "Transient = DQ 2×2 backward Euler (no LC filter). "
                            "Transient+LC = legacy ABC 6-state transient (LC filter). "
                            "Transient+LC-Full = DQ 6-state fully coupled sequential semi-implicit."
                        ),
                        "default": "Transient",
                    },
                    "periods": {
                        "type": "integer",
                        "description": "Number of fundamental periods to simulate.",
                        "default": 20,
                    },
                    "lc_filter": {
                        "type": "boolean",
                        "description": "Enable Y-connected LC output filter.",
                        "default": False,
                    },
                    "Lf": {
                        "type": "number",
                        "description": "Filter inductance per phase (henries).",
                        "default": 0.0004,
                    },
                    "Cf": {
                        "type": "number",
                        "description": "Filter capacitance per phase (farads), Y-connected.",
                        "default": 0.000025,
                    },
                    "grid_filter": {
                        "type": "boolean",
                        "description": "Enable grid-side DC-link ripple filter.",
                        "default": False,
                    },
                    "grid_Vpk": {
                        "type": "number",
                        "description": "Grid phase voltage amplitude (volts).",
                        "default": 245,
                    },
                    "grid_freq": {
                        "type": "number",
                        "description": "Grid frequency (Hz).",
                        "default": 50,
                    },
                    "dc_cap": {
                        "type": "number",
                        "description": "DC-link capacitance (farads).",
                        "default": 0.0035,
                    },
                    "dc_ind": {
                        "type": "number",
                        "description": "DC-link series inductance (henries).",
                        "default": 0.0005,
                    },
                    "dead_time_us": {
                        "type": "number",
                        "description": (
                            "Dead-time (blanking time) in microseconds. When > 0, "
                            "a DeadTimePWM decorator applies per-sample voltage error "
                            "ΔV_err = −sign(i_phase) · (td/Ts) · Vdc to simulate "
                            "inverter non-linearity. Typical IGBT: 1–4 µs, SiC MOSFET: "
                            "0.2–1 µs. Introduces characteristic 5th, 7th, 11th, 13th, … "
                            "harmonics. Default 0.0 (disabled)."
                        ),
                        "default": 0.0,
                    },
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "compute_operating_point",
            "description": (
                "Fast computation of PMSM steady-state operating point only. "
                "Uses DQ steady-state equations (no transient simulation). Returns Id, Iq, "
                "torque, power, and power factor. Use this when you only need "
                "the fundamental electrical quantities."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "poles": {
                        "type": "integer",
                        "description": "Number of rotor poles.",
                        "default": 8,
                    },
                    "R": {
                        "type": "number",
                        "description": "Phase resistance (ohms).",
                        "default": 0.15,
                    },
                    "Ld": {
                        "type": "number",
                        "description": "d-axis inductance (henries).",
                        "default": 0.0025,
                    },
                    "Lq": {
                        "type": "number",
                        "description": "q-axis inductance (henries).",
                        "default": 0.005,
                    },
                    "psi_pm": {
                        "type": "number",
                        "description": "PM flux linkage (webers).",
                        "default": 0.125,
                    },
                    "speed": {
                        "type": "number",
                        "description": "Rotor speed (RPM).",
                        "default": 3000,
                    },
                    "L_sigma_ratio": {
                        "type": "number",
                        "description": (
                            "Leakage inductance ratio Lσ/Ld (default 0.1). "
                            "Range 0.0–0.5."
                        ),
                        "default": 0.1,
                    },
                    "Vll": {
                        "type": "number",
                        "description": "Line-to-line peak voltage (volts).",
                        "default": 400,
                    },
                    "phase_deg": {
                        "type": "number",
                        "description": "Voltage angle in dq-frame (degrees).",
                        "default": 151.5,
                    },
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_waveforms",
            "description": (
                "Run the full PMSM drive simulation and return time-domain "
                "voltage and current waveforms (down-sampled to fit LLM context "
                "limits). Includes PWM phase voltages, line-to-line voltages, "
                "and phase currents."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "poles": {"type": "integer", "default": 8},
                    "R": {"type": "number", "default": 0.15},
                    "Ld": {"type": "number", "default": 0.0025},
                    "Lq": {"type": "number", "default": 0.005},
                    "psi_pm": {"type": "number", "default": 0.125},
                    "speed": {"type": "number", "default": 3000},
                    "L_sigma_ratio": {"type": "number", "default": 0.1},
                    "Vll": {"type": "number", "default": 400},
                    "phase_deg": {"type": "number", "default": 151.5},
                    "fsw": {"type": "number", "default": 8000},
                    "vdc": {"type": "number", "default": 400},
                    "mod": {
                        "type": "string",
                        "enum": [
                            "SPWM2",
                            "IPSPWM3",
                            "SVPWM2",
                            "SVPWM3",
                            "QuasiSVPWM2",
                            "QuasiSVPWM3",
                        ],
                        "default": "SVPWM2",
                    },
                    "third_harmonic": {"type": "boolean", "default": False},
                    "solver": {
                        "type": "string",
                        "enum": ["AC", "Transient", "Transient+LC", "Transient+LC-Full"],
                        "default": "Transient",
                    },
                    "periods": {"type": "integer", "default": 20},
                    "lc_filter": {"type": "boolean", "default": False},
                    "Lf": {"type": "number", "default": 0.0004},
                    "Cf": {"type": "number", "default": 0.000025},
                    "grid_filter": {"type": "boolean", "default": False},
                    "grid_Vpk": {"type": "number", "default": 245},
                    "grid_freq": {"type": "number", "default": 50},
                    "dc_cap": {"type": "number", "default": 0.0035},
                    "dc_ind": {"type": "number", "default": 0.0005},
                    "dead_time_us": {
                        "type": "number",
                        "description": (
                            "Dead-time (blanking time) in microseconds. When > 0, "
                            "a DeadTimePWM decorator applies per-sample voltage error "
                            "ΔV_err = −sign(i_phase) · (td/Ts) · Vdc to simulate "
                            "inverter non-linearity. Typical IGBT: 1–4 µs, SiC MOSFET: "
                            "0.2–1 µs. Introduces characteristic 5th, 7th, 11th, 13th, … "
                            "harmonics. Default 0.0 (disabled)."
                        ),
                        "default": 0.0,
                    },
                    "max_samples": {
                        "type": "integer",
                        "description": "Max waveform samples to return.",
                        "default": 500,
                    },
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_fft_spectra",
            "description": (
                "Run the full PMSM drive simulation and return FFT harmonic "
                "spectra for all voltage and current channels. Includes "
                "harmonic orders, amplitudes, and phase angles."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "poles": {"type": "integer", "default": 8},
                    "R": {"type": "number", "default": 0.15},
                    "Ld": {"type": "number", "default": 0.0025},
                    "Lq": {"type": "number", "default": 0.005},
                    "psi_pm": {"type": "number", "default": 0.125},
                    "speed": {"type": "number", "default": 3000},
                    "L_sigma_ratio": {"type": "number", "default": 0.1},
                    "Vll": {"type": "number", "default": 400},
                    "phase_deg": {"type": "number", "default": 151.5},
                    "fsw": {"type": "number", "default": 8000},
                    "vdc": {"type": "number", "default": 400},
                    "mod": {
                        "type": "string",
                        "enum": [
                            "SPWM2",
                            "IPSPWM3",
                            "SVPWM2",
                            "SVPWM3",
                            "QuasiSVPWM2",
                            "QuasiSVPWM3",
                        ],
                        "default": "SVPWM2",
                    },
                    "third_harmonic": {"type": "boolean", "default": False},
                    "solver": {
                        "type": "string",
                        "enum": ["AC", "Transient", "Transient+LC", "Transient+LC-Full"],
                        "default": "Transient",
                    },
                    "periods": {"type": "integer", "default": 20},
                    "lc_filter": {"type": "boolean", "default": False},
                    "Lf": {"type": "number", "default": 0.0004},
                    "Cf": {"type": "number", "default": 0.000025},
                    "grid_filter": {"type": "boolean", "default": False},
                    "grid_Vpk": {"type": "number", "default": 245},
                    "grid_freq": {"type": "number", "default": 50},
                    "dc_cap": {"type": "number", "default": 0.0035},
                    "dc_ind": {"type": "number", "default": 0.0005},
                    "dead_time_us": {
                        "type": "number",
                        "description": (
                            "Dead-time (blanking time) in microseconds. When > 0, "
                            "a DeadTimePWM decorator applies per-sample voltage error "
                            "ΔV_err = −sign(i_phase) · (td/Ts) · Vdc to simulate "
                            "inverter non-linearity. Typical IGBT: 1–4 µs, SiC MOSFET: "
                            "0.2–1 µs. Introduces characteristic 5th, 7th, 11th, 13th, … "
                            "harmonics. Default 0.0 (disabled)."
                        ),
                        "default": 0.0,
                    },
                    "max_harmonics": {
                        "type": "integer",
                        "description": "Max harmonic order to include.",
                        "default": 100,
                    },
                },
                "required": [],
            },
        },
    },
]


# ---------------------------------------------------------------------------
# Tool registry & dispatcher
# ---------------------------------------------------------------------------

_TOOL_REGISTRY: Dict[str, Any] = {
    "compute_pmsm_drive": compute_pmsm_drive,
    "compute_operating_point": compute_operating_point,
    "get_waveforms": get_waveforms,
    "get_fft_spectra": get_fft_spectra,
}


def get_tool_definitions() -> List[Dict[str, Any]]:
    """
    Return tool definitions in OpenAI function-calling format.

    Suitable for passing directly to an LLM API:

        tools = get_tool_definitions()
        response = client.chat.completions.create(
            model="gpt-4",
            messages=[...],
            tools=tools,
        )
    """
    return TOOL_DEFINITIONS


def run_tool(tool_name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
    """
    Dispatch a tool call by name with the given arguments.

    Usage:

        result = run_tool("compute_pmsm_drive", {
            "poles": 8, "speed": 3000, "Vll": 400, "phase_deg": 151.5
        })

    Parameters
    ----------
    tool_name : str
        One of: 'compute_pmsm_drive', 'compute_operating_point',
        'get_waveforms', 'get_fft_spectra'.
    arguments : dict
        Keyword arguments for the tool function. Unknown keys are silently
        ignored (allows passing full LLM-generated argument blobs).

    Returns
    -------
    dict
        Result dict from the tool function.

    Raises
    ------
    ValueError
        If tool_name is not recognised.
    """
    fn = _TOOL_REGISTRY.get(tool_name)
    if fn is None:
        available = list(_TOOL_REGISTRY)
        raise ValueError(
            f"Unknown tool '{tool_name}'. Available: {available}"
        )
    # Only pass arguments that the function accepts
    import inspect
    sig = inspect.signature(fn)
    valid_params = set(sig.parameters)
    filtered_args = {k: v for k, v in arguments.items() if k in valid_params}
    return fn(**filtered_args)


# ---------------------------------------------------------------------------
# CLI entry point (for direct invocation by LLM agents)
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="PMSM Drive Calculator — LLM Tool Interface",
    )
    sub = parser.add_subparsers(dest="tool", required=True)

    # compute_pmsm_drive sub-command
    p_drive = sub.add_parser("compute_pmsm_drive", help="Full simulation")
    _add_all_args(p_drive)
    p_drive.add_argument("--json", action="store_true",
                         help="Output raw JSON (default: pretty-printed)")

    # compute_operating_point sub-command
    p_op = sub.add_parser("compute_operating_point", help="Steady-state only")
    _add_motor_args(p_op)
    _add_voltage_basic_args(p_op)
    p_op.add_argument("--json", action="store_true")

    # get_waveforms sub-command
    p_wf = sub.add_parser("get_waveforms", help="Waveforms only")
    _add_all_args(p_wf)
    p_wf.add_argument("--max-samples", type=int, default=500)
    p_wf.add_argument("--json", action="store_true")

    # get_fft_spectra sub-command
    p_fft = sub.add_parser("get_fft_spectra", help="FFT spectra only")
    _add_all_args(p_fft)
    p_fft.add_argument("--max-harmonics", type=int, default=100)
    p_fft.add_argument("--json", action="store_true")

    args = parser.parse_args()
    tool_name = args.tool
    use_json = getattr(args, "json", False)

    # Build arguments dict from parsed args
    kwargs = vars(args).copy()
    for key in ["tool", "json"]:
        kwargs.pop(key, None)

    result = run_tool(tool_name, kwargs)

    if use_json:
        print(json.dumps(result))
    else:
        print(json.dumps(result, indent=2, default=str))


def _add_motor_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--poles", type=int, default=8)
    parser.add_argument("--R", type=float, default=0.15)
    parser.add_argument("--Ld", type=float, default=0.0025)
    parser.add_argument("--Lq", type=float, default=0.005)
    parser.add_argument("--psi-pm", type=float, default=0.125, dest="psi_pm")
    parser.add_argument("--speed", type=float, default=3000)
    parser.add_argument("--L-sigma-ratio", type=float, default=0.1,
                        dest="L_sigma_ratio",
                        help="Leakage inductance ratio Lσ/Ld (0.0–0.5)")


def _add_voltage_basic_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--Vll", type=float, default=400)
    parser.add_argument("--phase", type=float, default=151.5, dest="phase_deg")


def _add_all_args(parser: argparse.ArgumentParser) -> None:
    _add_motor_args(parser)
    _add_voltage_basic_args(parser)
    parser.add_argument("--fsw", type=float, default=8000)
    parser.add_argument("--vdc", type=float, default=400)
    parser.add_argument("--mod", type=str, default="SVPWM2",
                        choices=["SPWM2", "IPSPWM3", "SVPWM2", "SVPWM3"])
    parser.add_argument("--third-harmonic", action="store_true",
                        dest="third_harmonic")
    parser.add_argument("--solver", type=str, default="Transient",
                        choices=["AC", "Transient", "Transient+LC",
                                 "Transient+LC-Full"])
    parser.add_argument("--periods", type=int, default=20)
    parser.add_argument("--lc-filter", action="store_true", dest="lc_filter")
    parser.add_argument("--Lf", type=float, default=0.0004)
    parser.add_argument("--Cf", type=float, default=0.000025)
    parser.add_argument("--grid-filter", action="store_true", dest="grid_filter")
    parser.add_argument("--grid-Vpk", type=float, default=245, dest="grid_Vpk")
    parser.add_argument("--grid-freq", type=float, default=50, dest="grid_freq")
    parser.add_argument("--dc-cap", type=float, default=0.0035, dest="dc_cap")
    parser.add_argument("--dc-ind", type=float, default=0.0005, dest="dc_ind")
    parser.add_argument("--dead-time-us", type=float, default=0.0,
                        dest="dead_time_us",
                        help="Dead-time (blanking time) in microseconds")


if __name__ == "__main__":
    main()
