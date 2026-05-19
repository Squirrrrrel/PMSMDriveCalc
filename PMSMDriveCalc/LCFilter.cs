using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace PMSMDriveCalc
{
    /// <summary>
    /// 三相 PWM 输出端 LC 平波滤波器
    /// 拓扑：逆变器 PWM 输出 → 每相串联电感 L_f → 电机端口 + 星接(Y)电容 C_f 到中点
    /// 衰减开关频率纹波，平滑电机端电压
    /// </summary>
    public class OutputLCFilter : ICanOutputVoltage
    {
        private ICanOutputVoltage _innerPWM;
        private double _Lf;      // 每相串联滤波电感 (H)
        private double _Cf;      // 星接(Y)滤波电容 per phase (F)
        private double _R;       // 电机相电阻 (Ω)
        private double _Leq;     // 电机等效电感 at fundamental: (Ld+Lq)/2 (H)
        private double _Lsigma;  // 漏感 for high-frequency harmonics (H)
        private double _psiPM;   // 永磁磁链 (Wb)，用于计算反电势对电机端电压的贡献

        /// <summary>Raw PWM output (before LC filtering).</summary>
        public ICanOutputVoltage InnerPWM => _innerPWM;

        /// <summary>Filter inductance per phase (H).</summary>
        public double Lf => _Lf;
        /// <summary>Filter capacitance per phase (F), Y-connected.</summary>
        public double Cf => _Cf;
        /// <summary>Leakage inductance used for high-frequency impedance (H).</summary>
        public double Lsigma => _Lsigma;

        /// <param name="innerPWM">被包装的 PWM 模块</param>
        /// <param name="filterInductance">每相串联滤波电感 L_f (H)</param>
        /// <param name="filterCapacitance">星接滤波电容 C_f (F)，每相对中点</param>
        /// <param name="motorResistance">电机相电阻 R (Ω)</param>
        /// <param name="motorInductanceEq">电机等效电感 at fundamental L_eq = (Ld+Lq)/2 (H)</param>
        /// <param name="pmFluxLinkage">永磁磁链 ψ_pm (Wb)，用于基频反电势修正</param>
        /// <param name="lSigma">Leakage inductance for high-frequency harmonics (H). If 0, defaults to motorInductanceEq.</param>
        public OutputLCFilter(ICanOutputVoltage innerPWM, double filterInductance,
            double filterCapacitance, double motorResistance, double motorInductanceEq,
            double pmFluxLinkage = 0.0, double lSigma = 0.0)
        {
            _innerPWM = innerPWM;
            _Lf = filterInductance;
            _Cf = filterCapacitance;
            _R = motorResistance;
            _Leq = motorInductanceEq;
            _psiPM = pmFluxLinkage;
            _Lsigma = lSigma > 0 ? lSigma : motorInductanceEq;
        }

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
        /// LC 滤波器传递函数 H(jω) = V_motor / V_inv (逆变器电压源 → 电机端电压)
        /// 电路：V_inv → L_f → V_motor，负载 = 电机(Z_motor) ‖ C_f(星接电容)
        /// </summary>
        /// <param name="omega">Angular frequency (rad/s).</param>
        /// <param name="Z_motor">Motor impedance at this frequency.</param>
        private Complex TransferFunction(double omega, Complex Z_motor)
        {
            if (Math.Abs(omega) < 1e-12)
                return Complex.One;

            double Ceq = _Cf;
            Complex Z_C = new Complex(0, -1.0 / (omega * Ceq));
            Complex Z_parallel = (Z_motor * Z_C) / (Z_motor + Z_C);
            Complex Z_Lf = new Complex(0, omega * _Lf);
            return Z_parallel / (Z_Lf + Z_parallel);
        }

        /// <summary>
        /// Overload for backward compatibility: uses _Leq for motor impedance.
        /// </summary>
        private Complex TransferFunction(double omega)
        {
            return TransferFunction(omega, new Complex(_R, omega * _Leq));
        }

        /// <summary>
        /// 反电势 → 电机端电压的传递函数 H_emf(jω) = V_motor / E_back
        /// 叠加原理：短路 V_inv，反电势 E_back 经 Z_motor 串联，输出端 (Z_Lf ‖ Z_C) 分压
        /// 仅在基频 (ω = ω₁) 时有效，反电势不含高频分量
        /// </summary>
        private Complex BackEMFTransferFunction(double omega)
        {
            if (Math.Abs(omega) < 1e-12)
                return Complex.One;  // DC: 反电势不存在

            Complex Z_motor = new Complex(_R, omega * _Leq);
            Complex Z_C = new Complex(0, -1.0 / (omega * _Cf));
            Complex Z_Lf = new Complex(0, omega * _Lf);
            Complex Z_Lf_parallel_C = (Z_Lf * Z_C) / (Z_Lf + Z_C);
            return Z_Lf_parallel_C / (Z_motor + Z_Lf_parallel_C);
        }

        /// <summary>
        /// Compute motor terminal dq voltage after LC filter at a given frequency.
        /// V_motor = H(jω)·V_inv + H_emf(jω)·E_back
        /// Returns (motorUd, motorUq) in the dq frame.
        /// </summary>
        public (double MotorUd, double MotorUq) ComputeMotorDQVoltage(
            double Ud, double Uq, double omega, double pmFluxLinkage)
        {
            Complex H = TransferFunction(omega);
            Complex H_emf = BackEMFTransferFunction(omega);

            // Inverter dq voltage as complex: Ud (d-axis, real), Uq (q-axis, imag)
            Complex V_inv = new Complex(Ud, Uq);
            Complex V_motor = H * V_inv;

            // Back-EMF is on q-axis: E_back = ω·ψ_pm on +q axis (imag)
            Complex E_back = new Complex(0, omega * pmFluxLinkage);
            Complex V_motor_total = V_motor + H_emf * E_back;

            return (V_motor_total.Real, V_motor_total.Imaginary);
        }

        public List<List<double>> GetOutputVoltage(double amplitudeVoltage,
            double frequencyVoltage, double phaseVoltage, int periods)
        {
            // Step 1: 获取原始 PWM 输出电压
            var raw = _innerPWM.GetOutputVoltage(amplitudeVoltage, frequencyVoltage,
                phaseVoltage, periods);
            List<double> time = raw[0];

            // Step 2: 根据内层 PWM 输出通道数决定过滤模式
            if (raw.Count >= 4)
            {
                // 三相输出 (SVPWM2 / SVPWM3)：一次过滤三相
                // 反电势相位偏移：U=0, V=-2π/3, W=-4π/3 (三相正弦对称)
                List<double> uU_filtered = ApplyFilter(raw[1], time, frequencyVoltage, periods, 0.0);
                List<double> uV_filtered = ApplyFilter(raw[2], time, frequencyVoltage, periods,
                    -2.0 * Math.PI / 3.0);
                List<double> uW_filtered = ApplyFilter(raw[3], time, frequencyVoltage, periods,
                    -4.0 * Math.PI / 3.0);
                return new List<List<double>> { time, uU_filtered, uV_filtered, uW_filtered };
            }
            else
            {
                // 单相输出 (SPWM2 / DCFilteredPWM)：过滤单相波形
                // DQDriveCalculator 会为 U/V/W 三相分别调用，phaseVoltage 已带相位偏移
                List<double> u_filtered = ApplyFilter(raw[1], time, frequencyVoltage, periods,
                    phaseVoltage - (Math.PI / 2.0)); // phaseVoltage 是 sin 参考，转为 cos 参考
                return new List<List<double>> { time, u_filtered };
            }
        }

        /// <summary>
        /// 频域滤波：FFT → 按幅值排序取前 topK 谐波 → 施加 H(jω) → 时域重建
        /// 对基频分量叠加反电势贡献 V_motor = H·V_inv + H_emf·E_back(phaseOffset)
        /// </summary>
        private List<double> ApplyFilter(List<double> signal, List<double> time,
            double fFund, int periods, double backEmfPhaseOffset)
        {
            int N = signal.Count;
            double[] sigArray = signal.ToArray();

            // 正向 FFT
            FFTContainer fft = FFTOperations.GetFFT(sigArray);

            int numHarmonics = fft.Amplitude.Count;

            // First pass: apply H(jω) to ALL bins and compute filtered amplitude/phase.
            // This prevents harmonics near the LC resonance from being dropped prematurely:
            // they have tiny raw amplitude but get amplified ~14× by the filter.
            var allFiltered = new (double order, double filteredAmp, double filteredPhaseRad)[numHarmonics];
            for (int h = 0; h < numHarmonics; h++)
            {
                double order = fft.Order[h] / (double)periods;
                double omega = 2.0 * Math.PI * Math.Abs(order) * fFund;

                if (Math.Abs(order) < 1e-9)
                {
                    // DC component: pass through, convert phase to radians
                    allFiltered[h] = (order, fft.Amplitude[h], fft.Phase[h] * Math.PI / 180.0);
                }
                else
                {
                    bool isFund = Math.Abs(order - 1.0) < 1e-6;
                    // Use L_sigma for harmonics, L_eq for fundamental
                    Complex Z_motor = isFund
                        ? new Complex(_R, omega * _Leq)
                        : new Complex(_R, omega * _Lsigma);
                    Complex H = TransferFunction(omega, Z_motor);
                    if (isFund && _psiPM > 0)
                    {
                        // Fundamental: superposition V_motor = H·V_inv + H_emf·E_back
                        Complex V_inv = Complex.FromPolarCoordinates(fft.Amplitude[h],
                            fft.Phase[h] * Math.PI / 180.0);
                        Complex V_motor = H * V_inv;
                        // Back-EMF E_back: e(t) = ω₁·ψ_pm·sin(ω₁t + backEmfPhaseOffset)
                        // In cos reference: sin(θ) = cos(θ - π/2)
                        Complex H_emf = BackEMFTransferFunction(omega);
                        double E_amp = omega * _psiPM;
                        Complex E_back = Complex.FromPolarCoordinates(E_amp,
                            backEmfPhaseOffset - Math.PI / 2.0);
                        Complex V_motor_total = V_motor + H_emf * E_back;
                        allFiltered[h] = (order, V_motor_total.Magnitude, V_motor_total.Phase);
                    }
                    else
                    {
                        double filteredAmp = fft.Amplitude[h] * H.Magnitude;
                        double filteredPhaseRad = fft.Phase[h] * Math.PI / 180.0 + H.Phase;
                        allFiltered[h] = (order, filteredAmp, filteredPhaseRad);
                    }
                }
            }

            // Sort by FILTERED amplitude (descending), keep top K, then restore harmonic order
            int topK = Math.Min(200, numHarmonics);
            var topHarmonics = allFiltered
                .OrderByDescending(x => x.filteredAmp)
                .Take(topK)
                .OrderBy(x => x.order)
                .ToArray();

            double[] topOrders = new double[topK];
            double[] topAmps = new double[topK];
            double[] topPhases = new double[topK];
            for (int k = 0; k < topK; k++)
            {
                topOrders[k] = topHarmonics[k].order;
                topAmps[k] = topHarmonics[k].filteredAmp;
                topPhases[k] = topHarmonics[k].filteredPhaseRad;
            }

            // 仅用 topK 谐波快速重建时域
            double[] filtered = new double[N];
            for (int i = 0; i < N; i++)
            {
                double t_i = time[i];
                double sum = 0.0;
                for (int k = 0; k < topK; k++)
                {
                    double order = topOrders[k];
                    double freq = Math.Abs(order) * fFund;
                    double phase = topPhases[k];
                    if (order < -1e-9) phase = -phase; // 负频镜像
                    sum += topAmps[k] * Math.Cos(2.0 * Math.PI * freq * t_i + phase);
                }
                filtered[i] = sum;
            }

            return new List<double>(filtered);
        }

        /// <summary>
        /// Apply LC filter to a raw PWM FFT in the frequency domain, returning the motor voltage FFT.
        /// This avoids the double-FFT spectral leakage problem that occurs when the filtered
        /// time-domain signal (reconstructed from topK harmonics) is re-FFT'd for display.
        /// Each harmonic bin k has frequency ω = 2π·k·fFund, and the filter H(jω) is applied.
        /// For the fundamental (k=1), the back-EMF contribution H_emf·E_back is added.
        /// </summary>
        /// <param name="rawPwmFFT">FFT of the raw PWM signal (from a 1-fundamental-period window)</param>
        /// <param name="fFund">Fundamental frequency (Hz)</param>
        /// <param name="backEmfPhaseOffset">Phase offset for back-EMF (radians), e.g. 0 for U, -2π/3 for V, -4π/3 for W</param>
        /// <returns>FFTContainer with motor-side voltage spectrum (phase in degrees, matching FFTContainer convention)</returns>
        public FFTContainer GetOutputVoltageFFT(FFTContainer rawPwmFFT, double fFund, double backEmfPhaseOffset, int fftPeriods = 1)
        {
            int n = rawPwmFFT.Amplitude.Count;
            var motorOrders = new List<int>(n);
            var motorAmps = new List<double>(n);
            var motorPhases = new List<double>(n);

            for (int i = 0; i < n; i++)
            {
                int order = rawPwmFFT.Order[i];
                double omega = 2.0 * Math.PI * order * fFund / fftPeriods;

                if (order == 0)
                {
                    // DC component: passed through without filtering
                    motorOrders.Add(0);
                    motorAmps.Add(rawPwmFFT.Amplitude[i]);
                    motorPhases.Add(rawPwmFFT.Phase[i]);
                }
                else
                {
                    bool isFund = (order == fftPeriods);
                    // Use L_sigma for harmonics (order > 1), L_eq for fundamental
                    Complex Z_motor = isFund
                        ? new Complex(_R, omega * _Leq)
                        : new Complex(_R, omega * _Lsigma);
                    Complex H = TransferFunction(omega, Z_motor);

                    if (isFund && _psiPM > 0)
                    {
                        // Fundamental: V_motor = H·V_inv + H_emf·E_back
                        // V_inv phase from FFT is in degrees; convert to radians for complex arithmetic
                        Complex V_inv = Complex.FromPolarCoordinates(rawPwmFFT.Amplitude[i],
                            rawPwmFFT.Phase[i] * Math.PI / 180.0);
                        Complex V_motor = H * V_inv;

                        // Back-EMF: e(t) = ω₁·ψ_pm·sin(ω₁t + backEmfPhaseOffset)
                        // In cos reference: sin(θ) = cos(θ - π/2)
                        Complex H_emf = BackEMFTransferFunction(omega);
                        double E_amp = omega * _psiPM;
                        Complex E_back = Complex.FromPolarCoordinates(E_amp,
                            backEmfPhaseOffset - Math.PI / 2.0);
                        Complex V_motor_total = V_motor + H_emf * E_back;

                        motorOrders.Add(order);
                        motorAmps.Add(V_motor_total.Magnitude);
                        motorPhases.Add(V_motor_total.Phase * 180.0 / Math.PI); // radians → degrees
                    }
                    else
                    {
                        motorOrders.Add(order);
                        motorAmps.Add(rawPwmFFT.Amplitude[i] * H.Magnitude);
                        // Phase: input phase (degrees) + filter phase shift (radians) → degrees
                        double phaseDeg = rawPwmFFT.Phase[i] + H.Phase * 180.0 / Math.PI;
                        motorPhases.Add(phaseDeg);
                    }
                }
            }

            return new FFTContainer
            {
                Order = motorOrders,
                Amplitude = motorAmps,
                Phase = motorPhases
            };
        }

        /// <summary>
        /// Reconstruct a time-domain signal from its FFT content.
        /// Uses all harmonics (not just topK) for full-fidelity reconstruction.
        /// The phase in FFTContainer is in degrees; internally converted to radians.
        /// </summary>
        /// <param name="fft">FFT with orders, amplitudes, and phases (degrees).</param>
        /// <param name="time">Time vector (s).</param>
        /// <param name="fFund">Fundamental frequency (Hz).</param>
        /// <returns>Reconstructed time-domain signal.</returns>
        public List<double> ReconstructTimeDomainFromFFT(FFTContainer fft, List<double> time, double fFund)
        {
            int N = time.Count;
            double[] signal = new double[N];

            int numHarmonics = fft.Order.Count;
            for (int i = 0; i < N; i++)
            {
                double t_i = time[i];
                double sum = 0.0;
                for (int h = 0; h < numHarmonics; h++)
                {
                    double order = fft.Order[h];
                    double freq = Math.Abs(order) * fFund;
                    double amp = fft.Amplitude[h];
                    double phaseRad = fft.Phase[h] * Math.PI / 180.0;
                    if (order < -1e-9) phaseRad = -phaseRad; // negative frequency mirror
                    sum += amp * Math.Cos(2.0 * Math.PI * freq * t_i + phaseRad);
                }
                signal[i] = sum;
            }

            return new List<double>(signal);
        }
    }
}