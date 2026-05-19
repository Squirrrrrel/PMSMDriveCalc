using System;
using System.Collections.Generic;

namespace PMSMDriveCalc
{
    /// <summary>
    /// 电网侧模型：理想三相电网 → 二极管桥式整流 → DC 链路滤波器
    /// 产生含 6ω 纹波的 DC link 电压波形
    /// </summary>
    public class GridRectifierFilter
    {
        /// <summary>电网相电压幅值 (Vpeak)</summary>
        public double GridVoltageAmplitude { get; private set; }

        /// <summary>电网频率 (Hz)</summary>
        public double GridFrequency { get; private set; }

        /// <summary>DC 链路电容 (F)</summary>
        public double DCLinkCapacitance { get; private set; }

        /// <summary>DC 链路串联电感 (H), 0 表示纯电容滤波</summary>
        public double DCLinkInductance { get; private set; }

        /// <summary>DC 链路平均电流 (A), 由负载功率决定</summary>
        public double AverageDCCurrent { get; set; }

        /// <summary>理想空载平均 DC 电压 = 3√3/π * Vg_peak ≈ 1.35 * Vg_line_rms?</summary>
        public double IdealNoLoadDCAverage
        {
            get
            {
                // 6-pulse diode bridge: Vdc_avg = (3√3/π) * Vg_peak = (3/π) * Vg_line_peak
                // Vg_line_peak = √3 * GridVoltageAmplitude
                return 3.0 * Math.Sqrt(3.0) / Math.PI * GridVoltageAmplitude;
            }
        }

        /// <summary>DC 链路平均电压 (含负载压降)</summary>
        public double AverageDCVoltage
        {
            get
            {
                // 简化模型：Vdc_avg = Vdc_ideal - R_equiv * Idc
                // 其中等效电阻来自换相电感和二极管压降
                double commutationInductance = DCLinkInductance > 0 ? DCLinkInductance : 0.001 * GridVoltageAmplitude / (GridFrequency * 10.0);
                double vDrop = 6.0 * GridFrequency * commutationInductance * AverageDCCurrent + 2.0; // 2V diode drop
                return Math.Max(IdealNoLoadDCAverage - vDrop, 0.0);
            }
        }

        /// <summary>6ω 纹波电压峰值（简化解析模型）</summary>
        public double RipplePeakToPeak
        {
            get
            {
                // 纯电容滤波近似: ΔV ≈ Idc / (6·fg·C)
                if (DCLinkCapacitance > 1e-12)
                {
                    return AverageDCCurrent / (6.0 * GridFrequency * DCLinkCapacitance);
                }
                return 0.0;
            }
        }

        public GridRectifierFilter(double gridVoltageAmplitude, double gridFrequency,
            double dcLinkCapacitance, double dcLinkInductance = 0.0)
        {
            GridVoltageAmplitude = gridVoltageAmplitude;
            GridFrequency = gridFrequency;
            DCLinkCapacitance = dcLinkCapacitance;
            DCLinkInductance = dcLinkInductance;
            AverageDCCurrent = 0.0;
        }

        /// <summary>
        /// 获取给定时刻的 DC link 电压
        /// 模型：理想6脉波整流包络 + 电容滤波纹波
        /// Vdc(t) = Vdc_avg + (Vripple_pp/2) * sin(6·ωg·t + φ)
        /// </summary>
        public double GetDCLinkVoltage(double time)
        {
            double vdcAvg = AverageDCVoltage;
            double rippleAmp = RipplePeakToPeak / 2.0;
            // 6ω ripple phase: DC ripple minima align with grid zero-crossings
            double phase = -Math.PI / 2.0; // ripple minimum when rectified envelope dips
            double ripple = rippleAmp * Math.Sin(6.0 * 2.0 * Math.PI * GridFrequency * time + phase);
            return vdcAvg + ripple;
        }

        /// <summary>
        /// 获取完整时间序列的 DC link 电压波形
        /// </summary>
        public List<double> GetDCLinkVoltageWaveform(List<double> time)
        {
            List<double> vdc = new List<double>(time.Count);
            for (int i = 0; i < time.Count; i++)
            {
                vdc.Add(GetDCLinkVoltage(time[i]));
            }
            return vdc;
        }

        /// <summary>
        /// 设置负载条件（基于平均电机功率估算平均 DC 电流）
        /// </summary>
        public void SetLoadFromMotorPower(double motorPower, double efficiency = 0.95)
        {
            double vdcAvg = IdealNoLoadDCAverage;
            AverageDCCurrent = motorPower / (vdcAvg * efficiency);
        }

        /// <summary>
        /// 直接设置 DC 平均电流
        /// </summary>
        public void SetLoadCurrent(double dcCurrent)
        {
            AverageDCCurrent = dcCurrent;
        }
    }


    /// <summary>
    /// 含 DC link 纹波的 PWM 电压生成包装器
    /// 将恒 DC link 的 PWM 输出电压按实际 Vdc(t)/Vdc_avg 比例缩放
    /// 适用于 DC 纹波频率（~300Hz）远低于 PWM 开关频率（~8kHz）的场景
    /// </summary>
    public class DCFilteredPWM : ICanOutputVoltage
    {
        private ICanOutputVoltage _innerPWM;
        private GridRectifierFilter _gridFilter;
        private double _nominalDCLink;

        public DCFilteredPWM(ICanOutputVoltage innerPWM, GridRectifierFilter gridFilter, double nominalDCLink)
        {
            _innerPWM = innerPWM;
            _gridFilter = gridFilter;
            _nominalDCLink = nominalDCLink;
        }

        public double SwitchingFrequency
        {
            get
            {
                if (_innerPWM is PWM pwm)
                    return pwm.SwitchingFrequency;
                return 8000.0; // default
            }
        }

        public List<List<double>> GetOutputVoltage(double amplitudeVoltage, double frequencyVoltage,
            double phaseVoltage, int periods)
        {
            // Step 1: 获取恒 DC link 的 PWM 输出电压
            List<List<double>> nominalOutput = _innerPWM.GetOutputVoltage(amplitudeVoltage,
                frequencyVoltage, phaseVoltage, periods);

            List<double> time;
            List<double> vuNominal;
            List<double> vvNominal;
            List<double> vwNominal;

            if (nominalOutput.Count >= 4)
            {
                // 3-phase output (SVPWM2, SVPWM3)
                time = nominalOutput[0];
                vuNominal = nominalOutput[1];
                vvNominal = nominalOutput[2];
                vwNominal = nominalOutput[3];
            }
            else
            {
                // Single-phase output (SPWM2, IPSPWM3): generate each phase separately
                time = nominalOutput[0];
                vuNominal = nominalOutput[1];

                var phaseV = _innerPWM.GetOutputVoltage(amplitudeVoltage, frequencyVoltage,
                    phaseVoltage - 2.0 * Math.PI / 3.0, periods);
                var phaseW = _innerPWM.GetOutputVoltage(amplitudeVoltage, frequencyVoltage,
                    phaseVoltage - 4.0 * Math.PI / 3.0, periods);

                vvNominal = phaseV[1];
                vwNominal = phaseW[1];
            }

            // Step 2: 获取 DC link 电压波形
            List<double> vdc = _gridFilter.GetDCLinkVoltageWaveform(time);

            // Step 3: 按 Vdc(t)/Vdc_nominal 比例缩放各相电压
            List<double> vuFiltered = new List<double>(time.Count);
            List<double> vvFiltered = new List<double>(time.Count);
            List<double> vwFiltered = new List<double>(time.Count);

            for (int i = 0; i < time.Count; i++)
            {
                double scale = vdc[i] / _nominalDCLink;
                vuFiltered.Add(vuNominal[i] * scale);
                vvFiltered.Add(vvNominal[i] * scale);
                vwFiltered.Add(vwNominal[i] * scale);
            }

            return new List<List<double>> { time, vuFiltered, vvFiltered, vwFiltered };
        }
    }


    /// <summary>
    /// 简化的三相 LCL 电网滤波器 + 主动前端 (AFE) 模型
    /// 适用于 PWM 整流 + DC link 的完整建模
    /// </summary>
    public class AFEGridFilter
    {
        /// <summary>电网相电压幅值 (Vpeak)</summary>
        public double GridVoltageAmplitude { get; private set; }

        /// <summary>电网频率 (Hz)</summary>
        public double GridFrequency { get; private set; }

        /// <summary>网侧电感 (H)</summary>
        public double GridSideInductance { get; private set; }

        /// <summary>网侧电阻 (Ω)</summary>
        public double GridSideResistance { get; private set; }

        /// <summary>DC 链路电容 (F)</summary>
        public double DCLinkCapacitance { get; private set; }

        /// <summary>AFE 开关频率 (Hz)</summary>
        public double AFESwitchingFrequency { get; private set; }

        /// <summary>DC link 电压设定值 (V)</summary>
        public double DCLinkVoltageSetpoint { get; private set; }

        /// <summary>当前 DC 负载功率 (W)</summary>
        public double LoadPower { get; set; }

        public AFEGridFilter(double gridVoltageAmplitude, double gridFrequency,
            double gridSideInductance, double gridSideResistance,
            double dcLinkCapacitance, double afeSwitchingFrequency,
            double dcLinkVoltageSetpoint)
        {
            GridVoltageAmplitude = gridVoltageAmplitude;
            GridFrequency = gridFrequency;
            GridSideInductance = gridSideInductance;
            GridSideResistance = gridSideResistance;
            DCLinkCapacitance = dcLinkCapacitance;
            AFESwitchingFrequency = afeSwitchingFrequency;
            DCLinkVoltageSetpoint = dcLinkVoltageSetpoint;
            LoadPower = 0.0;
        }

        /// <summary>
        /// 简化 DC link 纹波模型：假设 AFE 控制良好，DC 电压主要含开关纹波
        /// Vdc(t) ≈ Vdc_setpoint + ΔV_sw · sin(2π·f_sw_afe·t)
        /// </summary>
        public double GetDCLinkVoltage(double time)
        {
            double dcCurrent = LoadPower / Math.Max(DCLinkVoltageSetpoint, 1.0);
            // 开关纹波近似
            double rippleSwitching = dcCurrent / (2.0 * AFESwitchingFrequency * DCLinkCapacitance);
            double rippleTotal = rippleSwitching;

            return DCLinkVoltageSetpoint +
                rippleTotal * Math.Sin(2.0 * Math.PI * AFESwitchingFrequency * time);
        }

        public List<double> GetDCLinkVoltageWaveform(List<double> time)
        {
            List<double> vdc = new List<double>(time.Count);
            for (int i = 0; i < time.Count; i++)
                vdc.Add(GetDCLinkVoltage(time[i]));
            return vdc;
        }
    }
}