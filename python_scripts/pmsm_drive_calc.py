#!/usr/bin/env python3
"""
PMSM Drive Calculator — Python API Wrapper
===========================================

Calls the PMSMDriveCalc .NET library via Python.NET (pythonnet) to perform
the same PMSM drive simulation as the GUI app.

Dependencies:
  pip install pythonnet numpy

The PMSMDriveCalc.dll must be built first:
  dotnet build PMSMDriveCalc/PMSMDriveCalc.csproj -c Release

Usage (CLI):
  python pmsm_drive_calc.py \\
      --poles 8 --R 0.15 --Ld 0.0025 --Lq 0.005 --psi_pm 0.125 \\
      --speed 3000 --Vll 400 --phase 151.5 \\
      --fsw 8000 --vdc 400 --mod SVPWM2 --solver Transient --periods 20

Usage (module):
  from pmsm_drive_calc import PMSMDriveCalcPython
  calc = PMSMDriveCalcPython(
      poles=8, R=0.15, Ld=0.0025, Lq=0.005, psi_pm=0.125, speed=3000
  )
  result = calc.run(
      Vll=400, phase_deg=151.5,
      fsw=8000, vdc=400, mod='SVPWM2',
      solver='Transient', periods=20
  )
  calc.print_result(result)
"""

from __future__ import annotations
import argparse
import csv
import os
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

# ── Python.NET setup ────────────────────────────────────────────────────────
# Determine the path to the compiled PMSMDriveCalc.dll.
# - If PYTHONNET_PMSM_DLL is set, use it directly.
# - Otherwise, look relative to this script (../../PMSMDriveCalc/bin/Release/net9.0/)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

if dll_path := os.environ.get("PYTHONNET_PMSM_DLL"):
    _DLL_PATH = dll_path
else:
    # Try Release first, then Debug
    _candidates = [
        os.path.join(_SCRIPT_DIR, "..", "PMSMDriveCalc", "bin", "Release", "net9.0"),
        os.path.join(_SCRIPT_DIR, "..", "PMSMDriveCalc", "bin", "Debug", "net9.0"),
    ]
    _DLL_DIR = None
    for c in _candidates:
        c = os.path.abspath(c)
        if os.path.isdir(c):
            _DLL_DIR = c
            break
    if _DLL_DIR is None:
        raise FileNotFoundError(
            "Cannot find PMSMDriveCalc.dll. Build with:\n"
            "  dotnet build PMSMDriveCalc/PMSMDriveCalc.csproj -c Release\n"
            "Or set PYTHONNET_PMSM_DLL env var to the full .dll path."
        )
    _DLL_PATH = os.path.join(_DLL_DIR, "PMSMDriveCalc.dll")
    if not os.path.isfile(_DLL_PATH):
        raise FileNotFoundError(f"DLL not found at {_DLL_PATH}. Build first.")

# pythonnet 3.x defaults to "mono" on macOS/Linux.
# If Mono is not installed, set PYTHONNET_RUNTIME=coreclr BEFORE importing clr.
import ctypes.util

if not os.environ.get("PYTHONNET_RUNTIME"):
    _mono_lib = ctypes.util.find_library("monosgen-2.0")
    if _mono_lib is None:
        os.environ["PYTHONNET_RUNTIME"] = "coreclr"

try:
    import clr
except ImportError:
    print("ERROR: pythonnet is not installed. Run:  pip install pythonnet")
    sys.exit(1)
except Exception as e:
    print(f"ERROR: Failed to load .NET CLR runtime: {e}")
    print("Try:  export PYTHONNET_RUNTIME=coreclr")
    sys.exit(1)

# CRITICAL: Remove the CWD from sys.path to prevent Python's import system
# from finding the local "PMSMDriveCalc/" source directory as a namespace
# package, which would shadow the CLR-loaded types.
if "" in sys.path:
    sys.path.remove("")

# Add DLL directory to sys.path for dependency resolution.
sys.path.insert(0, _DLL_DIR)

from System.Reflection import Assembly

# ── Load Meta.Numerics dependency first ──
# Costura.Fody embeds Meta.Numerics into the PMSMDriveCalc assembly, but
# Python.NET's Assembly.LoadFile does NOT trigger the embedded-resource
# loader.  We must load Meta.Numerics.dll explicitly from the NuGet cache.
_META_HOME = os.path.expanduser("~/.nuget/packages/meta.numerics")
_META_LOADED = False
if os.path.isdir(_META_HOME):
    _meta_candidates = sorted(os.listdir(_META_HOME), reverse=True)
    for _meta_ver in _meta_candidates:
        _meta_dll = os.path.join(_META_HOME, _meta_ver,
                                 "lib", "netstandard1.1", "Meta.Numerics.dll")
        if os.path.isfile(_meta_dll):
            try:
                _meta_asm = Assembly.LoadFile(_meta_dll)
                clr.AddReference(_meta_asm.GetName().Name)
                _META_LOADED = True
                break
            except Exception:
                continue

if not _META_LOADED:
    # Fallback: try to find Meta.Numerics.dll next to PMSMDriveCalc.dll
    _meta_fallback = os.path.join(_DLL_DIR, "Meta.Numerics.dll")
    if os.path.isfile(_meta_fallback):
        _meta_asm = Assembly.LoadFile(_meta_fallback)
        clr.AddReference(_meta_asm.GetName().Name)
        _META_LOADED = True

if not _META_LOADED:
    raise FileNotFoundError(
        "Cannot find Meta.Numerics.dll.  Either:\n"
        "  1. Run 'dotnet restore' to populate the NuGet cache, or\n"
        "  2. Copy Meta.Numerics.dll next to PMSMDriveCalc.dll"
    )

# Load the PMSMDriveCalc assembly.
_asm = Assembly.LoadFile(_DLL_PATH)
# Register with clr by assembly name (AddReference expects a string).
clr.AddReference(_asm.GetName().Name)
print(f"[pythonnet] Loaded: {_DLL_PATH}")

# Import .NET types into Python namespace
from System import Double, Int32, Math
from System.Collections.Generic import List as DotNetList
from PMSMDriveCalc import (
    PMSMdq,
    PMSMDQDriveCalculator,
    OperatingPointResult,
    PMSMDriveResult,
    SolverType,
    # PWM types
    SPWM2,
    IPSPWM3,
    SVPWM2,
    SVPWM3,
    QuasiSVPWM2,
    QuasiSVPWM3,
    ICanOutputVoltage,
    # Filters
    OutputLCFilter,
    GridRectifierFilter,
    DCFilteredPWM,
    # Dead-time effect
    DeadTimePWM,
    # Misc
    FFTContainer,
)

# ── Modulation name → constructor ──────────────────────────────────────────

_MODULATION_MAP: Dict[str, type] = {
    "SPWM2":        SPWM2,
    "IPSPWM3":      IPSPWM3,
    # "SVPWM2" and "SVPWM3" now dispatch to QuasiSVPWM internally (see API docs).
    "SVPWM2":       QuasiSVPWM2,
    "SVPWM3":       QuasiSVPWM3,
    # The original subdomain/sector-based SVPWM classes are preserved in source.
    "QuasiSVPWM2":  QuasiSVPWM2,
    "QuasiSVPWM3":  QuasiSVPWM3,
}

_SOLVER_MAP: Dict[str, SolverType] = {
    "AC":               SolverType.Transient,                     # Redirected: AC → DQ transient with initialized ICs
    "Transient":        SolverType.Transient,
    "Transient+LC":     SolverType.TransientWithLCFilter,         # LEGACY ABC 6-state
    "Transient+LC-Full": SolverType.TransientWithLCFilterFull,   # DQ 6-state fully coupled
}


# ── Data classes for result serialisation ───────────────────────────────────

@dataclass
class OperatingPoint:
    """Python-friendly mirror of OperatingPointResult (all values in SI units)."""
    Ud: float = 0.0
    Uq: float = 0.0
    Id: float = 0.0
    Iq: float = 0.0
    TorqueNm: float = 0.0
    PhaseVoltageMagnitude: float = 0.0
    VoltageAngleRad: float = 0.0
    CurrentMagnitude: float = 0.0
    CurrentAngleRad: float = 0.0
    ApparentPowerVA: float = 0.0
    PwmApparentPowerVA: float = 0.0
    ActivePowerW: float = 0.0
    PowerFactor: float = 0.0
    MechanicalPowerW: float = 0.0
    CopperLossW: float = 0.0
    MotorUd: float = 0.0
    MotorUq: float = 0.0
    ElectricalSpeedRadS: float = 0.0
    ElectricalFreqHz: float = 0.0
    SpeedRPM: float = 0.0
    Poles: int = 0
    PhaseResistance: float = 0.0
    Ld: float = 0.0
    Lq: float = 0.0
    PMFluxLinkage: float = 0.0
    LSigmaRatio: float = 0.1

    @staticmethod
    def from_dotnet(op: OperatingPointResult) -> "OperatingPoint":
        return OperatingPoint(
            Ud=op.Ud, Uq=op.Uq,
            Id=op.Id, Iq=op.Iq,
            TorqueNm=op.TorqueNm,
            PhaseVoltageMagnitude=op.PhaseVoltageMagnitude,
            VoltageAngleRad=op.VoltageAngleRad,
            CurrentMagnitude=op.CurrentMagnitude,
            CurrentAngleRad=op.CurrentAngleRad,
            ApparentPowerVA=op.ApparentPowerVA,
            PwmApparentPowerVA=op.PwmApparentPowerVA,
            ActivePowerW=op.ActivePowerW,
            PowerFactor=op.PowerFactor,
            MechanicalPowerW=op.MechanicalPowerW,
            CopperLossW=op.CopperLossW,
            MotorUd=op.MotorUd,
            MotorUq=op.MotorUq,
            ElectricalSpeedRadS=op.ElectricalSpeedRadS,
            ElectricalFreqHz=op.ElectricalFreqHz,
            SpeedRPM=op.SpeedRPM,
            Poles=op.Poles,
            PhaseResistance=op.PhaseResistance,
            Ld=op.Ld,
            Lq=op.Lq,
            PMFluxLinkage=op.PMFluxLinkage,
            LSigmaRatio=getattr(op, 'LSigmaRatio', 0.1),
        )


@dataclass
class FFTData:
    """Python-friendly mirror of FFTContainer."""
    orders: List[int] = field(default_factory=list)
    amplitudes: List[float] = field(default_factory=list)
    phases_deg: List[float] = field(default_factory=list)

    @staticmethod
    def from_dotnet(fft: Optional[FFTContainer]) -> Optional["FFTData"]:
        if fft is None or fft.Amplitude is None:
            return None
        return FFTData(
            orders=list(fft.Order),
            amplitudes=list(fft.Amplitude),
            phases_deg=list(fft.Phase),
        )


@dataclass
class ComputationResult:
    """Complete result of a PMSM drive simulation."""
    op: OperatingPoint
    time: List[float]
    # PWM phase voltages (inverter output, raw)
    VU: List[float]
    VV: List[float]
    VW: List[float]
    # Line-to-line PWM voltages
    VUV: List[float]
    VVW: List[float]
    VWU: List[float]
    # Phase currents
    IU: List[float]
    IV: List[float]
    IW: List[float]
    # Motor-side voltages (only populated when LC filter active)
    MotorVU: Optional[List[float]] = None
    MotorVV: Optional[List[float]] = None
    MotorVW: Optional[List[float]] = None
    MotorVUV: Optional[List[float]] = None
    MotorVVW: Optional[List[float]] = None
    MotorVWU: Optional[List[float]] = None
    # FFT data
    VU_FFT: Optional[FFTData] = None
    VV_FFT: Optional[FFTData] = None
    VW_FFT: Optional[FFTData] = None
    VUV_FFT: Optional[FFTData] = None
    VVW_FFT: Optional[FFTData] = None
    VWU_FFT: Optional[FFTData] = None
    IU_FFT: Optional[FFTData] = None
    IV_FFT: Optional[FFTData] = None
    IW_FFT: Optional[FFTData] = None
    MotorVU_FFT: Optional[FFTData] = None
    MotorVV_FFT: Optional[FFTData] = None
    MotorVW_FFT: Optional[FFTData] = None
    MotorVUV_FFT: Optional[FFTData] = None
    MotorVVW_FFT: Optional[FFTData] = None
    MotorVWU_FFT: Optional[FFTData] = None
    # Summary
    BaseFrequencyHz: float = 0.0
    PwmVPhasePhaseFundamental: float = 0.0
    MotorVPhaseNeutralRms: Optional[float] = None
    MotorVPhasePhaseFundamental: Optional[float] = None


# ── Main calculator class ───────────────────────────────────────────────────

class PMSMDriveCalcPython:
    """
    Python wrapper around PMSMDriveCalc .NET library.

    Mirrors the GUI app's pipeline: motor → PWM → (optional filters) → solver → FFT.

    Example
    -------
    >>> calc = PMSMDriveCalcPython(poles=8, R=0.15, Ld=0.0025, Lq=0.005, psi_pm=0.125, speed=3000)
    >>> result = calc.run(Vll=400, phase_deg=151.5, mod='SVPWM2', solver='Transient')
    >>> calc.print_result(result)
    """

    def __init__(
        self,
        poles: int = 8,
        R: float = 0.15,
        Ld: float = 0.0025,
        Lq: float = 0.005,
        psi_pm: float = 0.125,
        speed: float = 3000,
        L_sigma_ratio: float = 0.1,
    ):
        """
        Parameters
        ----------
        poles : int
            Number of rotor poles.
        R : float
            Stator phase resistance (Ohm).
        Ld : float
            d-axis inductance (H).
        Lq : float
            q-axis inductance (H).
        psi_pm : float
            Permanent-magnet flux linkage (Wb).
        speed : float
            Rotor speed (RPM).
        L_sigma_ratio : float
            Leakage/d-axis inductance ratio (Lσ/Ld) (default 0.1, clamped to 0.0–0.5).
        """
        self._poles = poles
        self._R = R
        self._Ld = Ld
        self._Lq = Lq
        self._psi_pm = psi_pm
        self._speed = speed
        self._L_sigma_ratio = L_sigma_ratio

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
        dead_time_us: float = 0.0,
        L_sigma_ratio: Optional[float] = None,
    ) -> ComputationResult:
        """
        Run the full PMSM drive simulation.

        Parameters
        ----------
        Vll : float
            Target line-to-line peak voltage (V).  For SVPWM this is
            clamped to vdc (the fundamental linear-modulation limit).
            V_PN_peak = Vll / √3.
        phase_deg : float
            Voltage phase angle in degrees (atan2(Uq, Ud)).
            Default 151.5° gives Ud≈83, Uq≈108 for Vll=400.
        fsw : float
            PWM switching frequency (Hz).
        vdc : float
            DC-link voltage (V).
        mod : str
            Modulation type: 'SPWM2', 'IPSPWM3', 'SVPWM2', 'SVPWM3',
            'QuasiSVPWM2', 'QuasiSVPWM3'.
            Note: 'SVPWM2' and 'SVPWM3' are internally implemented via
            QuasiSVPWM2/QuasiSVPWM3 (zero-sequence injection / saddle
            waveform), which is mathematically equivalent to traditional
            space-vector PWM but avoids the zero-order-hold (ZOH) phase lag
            (δθ = ω·Ts/2 ≈ 4.5° at 200 Hz, 8 kHz). The original
            SVPWM2/SVPWM3 classes are preserved but not used by the GUI.
        third_harmonic : bool
            Enable 1/6 third-harmonic injection (SPWM2/IPSPWM3 only).
        solver : str
            Solver type: 'Transient', 'Transient+LC', 'Transient+LC-Full'.
            (Note: 'AC' is accepted as an alias for 'Transient'
            — the frequency-domain AC solver has been replaced by the DQ transient
            solver with steady-state operating-point initialization.)
        periods : int
            Number of fundamental periods to simulate.
        lc_filter : bool
            Enable Y-connected LC output filter between inverter and motor.
        Lf : float
            Filter inductance per phase (H).
        Cf : float
            Filter capacitance per phase (F), Y-connected.
        grid_filter : bool
            Enable grid-side diode-bridge rectifier DC-link ripple filter.
        grid_Vpk : float
            Grid phase voltage amplitude (Vpeak ≈ 230 Vrms × √2).
        grid_freq : float
            Grid frequency (Hz).
        dc_cap : float
            DC-link capacitance (F).
        dc_ind : float
            DC-link series inductance (H).
        dead_time_us : float
            Dead-time (blanking time) in microseconds. When > 0, a DeadTimePWM
            decorator applies the per-sample voltage error ΔV_err = −sign(i_phase) ·
            (td/Ts) · Vdc to simulate inverter non-linearity.  Typical IGBT: 1–4 µs,
            SiC MOSFET: 0.2–1 µs.  Produces characteristic 5th, 7th, 11th, 13th, …
            harmonics in the motor current spectrum. Default 0.0 (disabled).
        L_sigma_ratio : float, optional
            Leakage/d-axis inductance ratio (Lσ/Ld). If None (default), uses the value
            set in __init__ (default 0.1). Range 0.0–0.5.

        Returns
        -------
        ComputationResult
            Operating point, time-domain waveforms, and FFT spectra.
        """
        lsigma = L_sigma_ratio if L_sigma_ratio is not None else self._L_sigma_ratio

        # ── Build motor ──
        motor = PMSMdq(self._poles, self._psi_pm, self._Ld, self._Lq, self._R, lsigma)
        motor.SpeedRPM = self._speed

        # ── Build PWM ──
        if mod not in _MODULATION_MAP:
            raise ValueError(f"Unknown modulation '{mod}'. Options: {list(_MODULATION_MAP)}")

        pwm_cls = _MODULATION_MAP[mod]
        third_val = 1 if third_harmonic else 0

        if pwm_cls is SPWM2:
            pwm = SPWM2(fsw, vdc, third_val)
        elif pwm_cls is IPSPWM3:
            pwm = IPSPWM3(fsw, vdc, third_val)
        elif pwm_cls is SVPWM2:
            pwm = SVPWM2(fsw, vdc)
        elif pwm_cls is SVPWM3:
            pwm = SVPWM3(fsw, vdc)
        elif pwm_cls is QuasiSVPWM2:
            pwm = QuasiSVPWM2(fsw, vdc)
        elif pwm_cls is QuasiSVPWM3:
            pwm = QuasiSVPWM3(fsw, vdc)
        else:
            raise ValueError(f"Unsupported PWM: {pwm_cls}")

        final_output: ICanOutputVoltage = pwm

        # ── Dead-time effect (applies per-sample voltage error before other decorators) ──
        if dead_time_us > 0:
            final_output = DeadTimePWM(final_output, motor, dead_time_us)

        # ── Grid-side DC-link ripple filter ──
        if grid_filter:
            grid = GridRectifierFilter(grid_Vpk, grid_freq, dc_cap, dc_ind)
            est_power = 5000.0  # nominal estimate
            grid.SetLoadFromMotorPower(est_power)
            final_output = DCFilteredPWM(final_output, grid, vdc)

        # ── Motor-side LC output filter ──
        if lc_filter:
            leq = (self._Ld + self._Lq) / 2.0
            lsig = lsigma * self._Ld
            final_output = OutputLCFilter(
                final_output, Lf, Cf, self._R, leq, self._psi_pm, lsig
            )

        # ── DQ voltage from polar coords ──
        Vpn_peak = Vll / Math.Sqrt(3.0)
        phase_rad = phase_deg * Math.PI / 180.0
        Ud = Vpn_peak * Math.Cos(phase_rad)
        Uq = Vpn_peak * Math.Sin(phase_rad)

        # ── Solver type ──
        if solver not in _SOLVER_MAP:
            raise ValueError(f"Unknown solver '{solver}'. Options: {list(_SOLVER_MAP)}")
        st = _SOLVER_MAP[solver]

        # ── Compute ──
        calc = PMSMDQDriveCalculator(motor, final_output)
        dr: PMSMDriveResult = calc.Compute(Ud, Uq, st, periods)

        # ── Extract into Python-friendly result ──
        return self._build_result(dr)

    # ── Helper: convert .NET PMSMDriveResult → ComputationResult ─────────

    @staticmethod
    def _list_to_native(net_list) -> List[float]:
        """Convert .NET List<double> to Python list[float]."""
        if net_list is None:
            return []
        return [net_list[i] for i in range(net_list.Count)]

    @staticmethod
    def _optional_list(net_list) -> Optional[List[float]]:
        if net_list is None or net_list.Count == 0:
            return None
        return PMSMDriveCalcPython._list_to_native(net_list)

    @classmethod
    def _build_result(cls, dr: PMSMDriveResult) -> ComputationResult:
        return ComputationResult(
            op=OperatingPoint.from_dotnet(dr.OperatingPoint),
            time=cls._list_to_native(dr.Time),
            VU=cls._list_to_native(dr.VU),
            VV=cls._list_to_native(dr.VV),
            VW=cls._list_to_native(dr.VW),
            VUV=cls._list_to_native(dr.VUV),
            VVW=cls._list_to_native(dr.VVW),
            VWU=cls._list_to_native(dr.VWU),
            IU=cls._list_to_native(dr.IU),
            IV=cls._list_to_native(dr.IV),
            IW=cls._list_to_native(dr.IW),
            MotorVU=cls._optional_list(dr.MotorVU),
            MotorVV=cls._optional_list(dr.MotorVV),
            MotorVW=cls._optional_list(dr.MotorVW),
            MotorVUV=cls._optional_list(dr.MotorVUV),
            MotorVVW=cls._optional_list(dr.MotorVVW),
            MotorVWU=cls._optional_list(dr.MotorVWU),
            VU_FFT=FFTData.from_dotnet(dr.VU_FFT),
            VV_FFT=FFTData.from_dotnet(dr.VV_FFT),
            VW_FFT=FFTData.from_dotnet(dr.VW_FFT),
            VUV_FFT=FFTData.from_dotnet(dr.VUV_FFT),
            VVW_FFT=FFTData.from_dotnet(dr.VVW_FFT),
            VWU_FFT=FFTData.from_dotnet(dr.VWU_FFT),
            IU_FFT=FFTData.from_dotnet(dr.IU_FFT),
            IV_FFT=FFTData.from_dotnet(dr.IV_FFT),
            IW_FFT=FFTData.from_dotnet(dr.IW_FFT),
            MotorVU_FFT=FFTData.from_dotnet(dr.MotorVU_FFT),
            MotorVV_FFT=FFTData.from_dotnet(dr.MotorVV_FFT),
            MotorVW_FFT=FFTData.from_dotnet(dr.MotorVW_FFT),
            MotorVUV_FFT=FFTData.from_dotnet(dr.MotorVUV_FFT),
            MotorVVW_FFT=FFTData.from_dotnet(dr.MotorVVW_FFT),
            MotorVWU_FFT=FFTData.from_dotnet(dr.MotorVWU_FFT),
            BaseFrequencyHz=dr.BaseFrequencyHz,
            PwmVPhasePhaseFundamental=dr.PwmVPhasePhaseFundamental,
            MotorVPhaseNeutralRms=(
                float(dr.MotorVPhaseNeutralRms)
                if dr.MotorVPhaseNeutralRms is not None
                else None
            ),
            MotorVPhasePhaseFundamental=(
                float(dr.MotorVPhasePhaseFundamental)
                if dr.MotorVPhasePhaseFundamental is not None
                else None
            ),
        )

    # ── Printing ────────────────────────────────────────────────────────

    @staticmethod
    def print_result(r: ComputationResult) -> None:
        """Print a human-readable summary of the computation result."""
        op = r.op
        sep = "─" * 60
        pi = Math.PI

        print("\n" + sep)
        print("  PMSM Drive Calculation Result")
        print(sep)

        print(f"\n  ── Motor ──")
        print(f"  Poles              : {op.Poles}")
        print(f"  Speed              : {op.SpeedRPM:.1f} RPM")
        print(f"  Electrical freq    : {op.ElectricalFreqHz:.2f} Hz")
        print(f"  Phase resistance   : {op.PhaseResistance:.4f} Ω")
        print(f"  Ld / Lq            : {op.Ld:.6f} / {op.Lq:.6f} H")
        print(f"  PM flux linkage    : {op.PMFluxLinkage:.4f} Wb")

        print(f"\n  ── Voltage Input (target) ──")
        print(f"  V_LL peak          : {op.PhaseVoltageMagnitude * Math.Sqrt(3.0):.2f} V")
        print(f"  V_PN peak          : {op.PhaseVoltageMagnitude:.2f} V")
        print(f"  Ud / Uq            : {op.Ud:.4f} / {op.Uq:.4f} V")
        print(f"  Voltage angle      : {op.VoltageAngleRad * 180.0 / pi:.2f}°")

        print(f"\n  ── Currents ──")
        print(f"  Id                 : {op.Id:.4f} A")
        print(f"  Iq                 : {op.Iq:.4f} A")
        print(f"  Current magnitude  : {op.CurrentMagnitude:.4f} A")
        print(f"  Current angle      : {op.CurrentAngleRad * 180.0 / pi:.2f}°")

        print(f"\n  ── Torque & Power ──")
        print(f"  Torque             : {op.TorqueNm:.4f} Nm")
        print(f"  Mechanical power   : {op.MechanicalPowerW:.2f} W")
        print(f"  Active power       : {op.ActivePowerW:.2f} W")
        print(f"  Apparent power     : {op.ApparentPowerVA:.2f} VA")
        if abs(op.PwmApparentPowerVA - op.ApparentPowerVA) > 0.01:
            print(f"  Inv. apparent pwr  : {op.PwmApparentPowerVA:.2f} VA")
        print(f"  Power factor       : {op.PowerFactor:.4f}")
        print(f"  Copper loss        : {op.CopperLossW:.2f} W")

        print(f"\n  ── Actual motor terminal voltages ──")
        print(f"  Motor Ud / Uq      : {op.MotorUd:.4f} / {op.MotorUq:.4f} V")
        motor_vmag = Math.Sqrt(op.MotorUd * op.MotorUd + op.MotorUq * op.MotorUq)
        print(f"  Motor |V_dq|       : {motor_vmag:.4f} V (peak phase)")

        print(f"\n  ── PWM ──")
        print(f"  PWM V_LL fund      : {r.PwmVPhasePhaseFundamental:.4f} V (peak)")
        if r.MotorVPhaseNeutralRms is not None:
            print(f"  Motor V_PN RMS     : {r.MotorVPhaseNeutralRms:.4f} V")
        if r.MotorVPhasePhaseFundamental is not None:
            print(f"  Motor V_LL fund    : {r.MotorVPhasePhaseFundamental:.4f} V (peak)")

        print(f"\n  ── Waveforms ──")
        print(f"  Time points        : {len(r.time)}")
        print(f"  Duration           : {r.time[-1] if r.time else 0:.4f} s")
        print(f"  Base frequency     : {r.BaseFrequencyHz:.2f} Hz")

        print(sep + "\n")

    @staticmethod
    def export_waveforms_csv(r: ComputationResult, filename: str) -> None:
        """
        Export time-domain waveforms to CSV.
        Columns: time, VU, VV, VW, VUV, VVW, VWU, IU, IV, IW
        (plus MotorVU, MotorVV, MotorVW if LC filter active).
        """
        n = len(r.time)
        if n == 0:
            print("No waveform data to export.")
            return

        headers = ["time", "VU", "VV", "VW", "VUV", "VVW", "VWU", "IU", "IV", "IW"]
        if r.MotorVU is not None:
            headers += ["MotorVU", "MotorVV", "MotorVW",
                        "MotorVUV", "MotorVVW", "MotorVWU"]

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            for i in range(n):
                row = [
                    r.time[i], r.VU[i], r.VV[i], r.VW[i],
                    r.VUV[i], r.VVW[i], r.VWU[i],
                    r.IU[i], r.IV[i], r.IW[i],
                ]
                if r.MotorVU is not None:
                    row += [
                        r.MotorVU[i], r.MotorVV[i], r.MotorVW[i],
                        r.MotorVUV[i], r.MotorVVW[i], r.MotorVWU[i],
                    ]
                writer.writerow(row)

        print(f"Waveforms exported to {filename} ({n} rows)")

    @staticmethod
    def export_fft_csv(r: ComputationResult, filename: str) -> None:
        """
        Export FFT data to CSV.
        Columns: order, IU_amp, IU_phase, IV_amp, IV_phase, IW_amp, IW_phase
        """
        if r.IU_FFT is None:
            print("No FFT data to export.")
            return

        orders = r.IU_FFT.orders
        n = len(orders)

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "order",
                "IU_amp", "IU_phase_deg",
                "IV_amp", "IV_phase_deg",
                "IW_amp", "IW_phase_deg",
                "VU_amp", "VU_phase_deg",
            ])
            for i in range(n):
                row = [orders[i]]
                if r.IU_FFT and i < len(r.IU_FFT.amplitudes):
                    row += [r.IU_FFT.amplitudes[i], r.IU_FFT.phases_deg[i]]
                else:
                    row += ["", ""]
                if r.IV_FFT and i < len(r.IV_FFT.amplitudes):
                    row += [r.IV_FFT.amplitudes[i], r.IV_FFT.phases_deg[i]]
                else:
                    row += ["", ""]
                if r.IW_FFT and i < len(r.IW_FFT.amplitudes):
                    row += [r.IW_FFT.amplitudes[i], r.IW_FFT.phases_deg[i]]
                else:
                    row += ["", ""]
                if r.VU_FFT and i < len(r.VU_FFT.amplitudes):
                    row += [r.VU_FFT.amplitudes[i], r.VU_FFT.phases_deg[i]]
                else:
                    row += ["", ""]
                writer.writerow(row)

        print(f"FFT exported to {filename} ({n} bins)")


# ── CLI entry point ─────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="PMSM Drive Calculator — Python CLI (Python.NET bridge to PMSMDriveCalc.dll)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # SVPWM 2-level, transient solver, 20 periods (defaults)
  python pmsm_drive_calc.py

  # SPWM with 3rd harmonic, AC solver
  python pmsm_drive_calc.py --mod SPWM2 --third-harmonic --solver AC

  # With LC output filter
  python pmsm_drive_calc.py --lc-filter --solver Transient+LC --Lf 0.0005 --Cf 0.00002

  # Export waveforms to CSV
  python pmsm_drive_calc.py --export-csv result.csv

  # Custom motor
  python pmsm_drive_calc.py --poles 4 --R 0.2 --Ld 0.001 --Lq 0.0018 --psi-pm 0.08 --speed 5000
""",
    )

    # Motor
    mot = parser.add_argument_group("Motor parameters")
    mot.add_argument("--poles", type=int, default=8, help="Number of poles (default: 8)")
    mot.add_argument("--R", type=float, default=0.15, help="Phase resistance Ω (default: 0.15)")
    mot.add_argument("--Ld", type=float, default=0.0025, help="d-axis inductance H (default: 0.0025)")
    mot.add_argument("--Lq", type=float, default=0.005, help="q-axis inductance H (default: 0.005)")
    mot.add_argument("--psi-pm", type=float, default=0.125, help="PM flux linkage Wb (default: 0.125)")
    mot.add_argument("--speed", type=float, default=3000, help="Rotor speed RPM (default: 3000)")
    mot.add_argument("--L-sigma-ratio", type=float, default=0.1,
                     help="Leakage/d-axis inductance ratio (Lσ/Ld) (default: 0.1, range 0.0–0.5)")

    # Voltage
    vol = parser.add_argument_group("Voltage / Inverter")
    vol.add_argument("--Vll", type=float, default=400.0,
                     help="Target line-to-line peak voltage V (default: 400)")
    vol.add_argument("--phase", type=float, default=151.5,
                     help="Voltage phase angle ° (default: 151.5)")
    vol.add_argument("--fsw", type=float, default=8000.0,
                     help="Switching frequency Hz (default: 8000)")
    vol.add_argument("--vdc", type=float, default=400.0,
                     help="DC-link voltage V (default: 400)")
    vol.add_argument("--mod", type=str, default="SVPWM2",
                     choices=list(_MODULATION_MAP),
                     help="Modulation type: SPWM2, IPSPWM3, SVPWM2, SVPWM3, QuasiSVPWM2, QuasiSVPWM3 (default: SVPWM2). SVPWM2/SVPWM3 internally dispatch to QuasiSVPWM for ZOH phase-lag avoidance.")
    vol.add_argument("--third-harmonic", action="store_true",
                     help="Enable 1/6 third-harmonic injection (SPWM2/IPSPWM3 only)")

    # Solver
    sol = parser.add_argument_group("Solver")
    sol.add_argument("--solver", type=str, default="Transient",
                     choices=list(_SOLVER_MAP),
                     help="Solver type (default: Transient)")
    sol.add_argument("--periods", type=int, default=20,
                     help="Fundamental periods to simulate (default: 20)")

    # LC filter
    lc = parser.add_argument_group("LC Output Filter")
    lc.add_argument("--lc-filter", action="store_true",
                    help="Enable Y-connected LC output filter")
    lc.add_argument("--Lf", type=float, default=0.0004,
                    help="Filter inductance per phase H (default: 0.0004)")
    lc.add_argument("--Cf", type=float, default=0.000025,
                    help="Filter capacitance per phase F, Y-connected (default: 2.5e-5)")

    # Grid filter
    gf = parser.add_argument_group("Grid DC-Link Ripple Filter")
    gf.add_argument("--grid-filter", action="store_true",
                    help="Enable grid-side diode rectifier DC ripple filter")
    gf.add_argument("--grid-Vpk", type=float, default=245.0,
                    help="Grid phase voltage amplitude V (default: 245 ≈ 230 Vrms × √2)")
    gf.add_argument("--grid-freq", type=float, default=50.0,
                    help="Grid frequency Hz (default: 50)")
    gf.add_argument("--dc-cap", type=float, default=0.0035,
                    help="DC-link capacitance F (default: 0.0035)")
    gf.add_argument("--dc-ind", type=float, default=0.0005,
                    help="DC-link series inductance H (default: 0.0005)")

    # Output
    out = parser.add_argument_group("Output")
    out.add_argument("--export-csv", type=str, default=None,
                     help="Export time-domain waveforms to CSV file")
    out.add_argument("--export-fft-csv", type=str, default=None,
                     help="Export FFT spectra to CSV file")
    out.add_argument("--quiet", action="store_true",
                     help="Suppress summary printout")

    args = parser.parse_args()

    # ── Create calculator and run ──
    calc = PMSMDriveCalcPython(
        poles=args.poles,
        R=args.R,
        Ld=args.Ld,
        Lq=args.Lq,
        psi_pm=args.psi_pm,
        speed=args.speed,
        L_sigma_ratio=args.L_sigma_ratio,
    )

    try:
        result = calc.run(
            Vll=args.Vll,
            phase_deg=args.phase,
            fsw=args.fsw,
            vdc=args.vdc,
            mod=args.mod,
            third_harmonic=args.third_harmonic,
            solver=args.solver,
            periods=args.periods,
            lc_filter=args.lc_filter,
            Lf=args.Lf,
            Cf=args.Cf,
            grid_filter=args.grid_filter,
            grid_Vpk=args.grid_Vpk,
            grid_freq=args.grid_freq,
            dc_cap=args.dc_cap,
            dc_ind=args.dc_ind,
        )
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)

    # ── Output ──
    if not args.quiet:
        calc.print_result(result)

    if args.export_csv:
        calc.export_waveforms_csv(result, args.export_csv)

    if args.export_fft_csv:
        calc.export_fft_csv(result, args.export_fft_csv)


if __name__ == "__main__":
    main()
