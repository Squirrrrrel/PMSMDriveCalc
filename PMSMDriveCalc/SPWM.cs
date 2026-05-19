using System;
using System.Collections.Generic;
using System.Numerics;



namespace PMSMDriveCalc
{

    public class PWM
    {
        public double SwitchingFrequency { get; private set; }
        public double DCLink { get; private set; }

        public PWM(double switchingFrequency, double dCLink)
        {
            SwitchingFrequency = switchingFrequency;
            DCLink = dCLink;
        }

    }

    public interface ICanOutputVoltage
    {
        List<List<double>> GetOutputVoltage(double amplitudeVoltage, double frequencyVoltage,
            double phaseVoltage, int periods);
    }


    public class IPSPWM3 : PWM, ICanOutputVoltage
    {
        private int thirdHarmonicInjection;
        public IPSPWM3(double switchingFrequency, double dCLink, int thirdHarmonicInjection) :
            base(switchingFrequency, dCLink)
        {
            this.thirdHarmonicInjection = thirdHarmonicInjection;
        }

        public List<List<double>> GetOutputVoltage(double amplitudeVoltage, double frequencyVoltage,
            double phaseVoltage, int periods)
        {
            double timeMax = periods / frequencyVoltage;
            List<double> driverUpper = new List<double> { };
            List<double> driverLower = new List<double> { };
            List<double> time = new List<double> { };

            for (int i = 0; i < 50; i++)
            {
                driverUpper.Add(this.DCLink / 4.0 - this.DCLink / 4.0 / 50.0 * i);
            }

            for (int i = 0; i < 100; i++)
            {
                driverUpper.Add(this.DCLink / 2.0 / 100.0 * i);
            }

            for (int i = 0; i < 50; i++)
            {
                driverUpper.Add(this.DCLink / 2.0 - this.DCLink / 4.0 / 50.0 * i);
            }

            for (int i = 0; i < 200; i++)
            {
                time.Add(1.0 / this.SwitchingFrequency / 200.0 * i);
                driverLower.Add(driverUpper[i] - this.DCLink / 2.0);
            }

            driverUpper.Add(this.DCLink / 4.0);
            driverLower.Add(-this.DCLink / 4.0);

            double factor = 0.0;
            if (thirdHarmonicInjection == 1) factor = 1.0 / 6.0;
            Func<double, double> u = t => amplitudeVoltage * Math.Sin(2.0 * Math.PI *
                frequencyVoltage * t + phaseVoltage) + factor * amplitudeVoltage *
                Math.Sin(6.0 * Math.PI * frequencyVoltage * t + 3.0 * phaseVoltage);

            List<double> ret_voltage = new List<double> { };
            List<double> ret_time = new List<double> { };

            for (int i = 0; i < int.MaxValue; i++)
            {
                int _i = i % 200;
                double _t = i * 1.0 / this.SwitchingFrequency / 200.0;
                if (_t <= timeMax) ret_time.Add(_t);
                else break;
                double _u = u(_t);
                double _driverUpper = driverUpper[_i];
                double _driverLower = driverLower[_i];

                if (_u > _driverUpper) ret_voltage.Add(this.DCLink / 2.0);
                else if (_u < _driverLower) ret_voltage.Add(-this.DCLink / 2.0);
                else ret_voltage.Add(0.0);
            }

            return new List<List<double>> { ret_time, ret_voltage };
        }
    }


    public class SPWM2 : PWM, ICanOutputVoltage
    {
        public SPWM2(double switchingFrequency, double dCLink, int thirdHarmonicInjection) :
            base(switchingFrequency, dCLink)
        {
            this.thirdHarmonicInjection = thirdHarmonicInjection;
        }

        private int thirdHarmonicInjection;

        public List<List<double>> GetOutputVoltage(double amplitudeVoltage, double frequencyVoltage,
            double phaseVoltage, int periods)
        {
            double timeMax = periods / frequencyVoltage;
            List<double> driver = new List<double> { };
            List<double> time = new List<double> { };

            for (int i = 0; i < 50; i++)
            {
                driver.Add( - this.DCLink / 2.0 / 50.0 * i);
            }

            for (int i = 0; i < 100; i++)
            {
                driver.Add( - this.DCLink / 2.0 + this.DCLink / 100.0 * i);
            }

            for (int i = 0; i < 50; i++)
            {
                driver.Add(this.DCLink / 2.0 - this.DCLink / 2.0 / 50.0 * i);
            }

            for (int i = 0; i < 200; i++)
            {
                time.Add(1.0 / this.SwitchingFrequency / 200.0 * i);
            }
            driver.Add(0.0);

            double factor = 0.0;
            if (thirdHarmonicInjection == 1) factor = 1.0 / 6.0;
            Func<double, double> u = t => amplitudeVoltage * Math.Sin(2.0 * Math.PI *
                frequencyVoltage * t + phaseVoltage) + factor * amplitudeVoltage *
                Math.Sin(6.0 * Math.PI * frequencyVoltage * t + 3.0 * phaseVoltage);

            List<double> ret_voltage = new List<double> { };
            List<double> ret_time = new List<double> { };

            for (int i = 0; i < int.MaxValue; i++)
            {
                int _i = i % 200;
                double _t = i * 1.0 / this.SwitchingFrequency / 200.0;
                if (_t <= timeMax) ret_time.Add(_t);
                else break;
                double _u = u(_t);
                double _driver = driver[_i];

                if (_u >= _driver) ret_voltage.Add(this.DCLink / 2.0);
                else ret_voltage.Add(-this.DCLink / 2.0);
            }

            return new List<List<double>> { ret_time, ret_voltage };
        }

    }


    public class QuasiSVPWM3 : PWM, ICanOutputVoltage
    {
        public QuasiSVPWM3(double switchingFrequency, double dCLink) :
            base(switchingFrequency, dCLink)
        {
        }

        public List<List<double>> GetOutputVoltage(double amplitudeVoltage, double frequencyVoltage,
            double phaseVoltage, int periods)
        {
            double timeMax = periods / frequencyVoltage;
            List<double> driverUpper = new List<double> { };
            List<double> driverLower = new List<double> { };
            List<double> time = new List<double> { };

            // Build upper triangular carrier (0 → Vdc/2), 200 points per switching period
            for (int i = 0; i < 50; i++)
            {
                driverUpper.Add(this.DCLink / 4.0 - this.DCLink / 4.0 / 50.0 * i);
            }

            for (int i = 0; i < 100; i++)
            {
                driverUpper.Add(this.DCLink / 2.0 / 100.0 * i);
            }

            for (int i = 0; i < 50; i++)
            {
                driverUpper.Add(this.DCLink / 2.0 - this.DCLink / 4.0 / 50.0 * i);
            }

            for (int i = 0; i < 200; i++)
            {
                time.Add(1.0 / this.SwitchingFrequency / 200.0 * i);
                driverLower.Add(driverUpper[i] - this.DCLink / 2.0);
            }

            driverUpper.Add(this.DCLink / 4.0);
            driverLower.Add(-this.DCLink / 4.0);

            // 3-phase sinusoidal references (computed internally for zero-sequence extraction).
            // The zero-sequence component is invariant to cyclic permutation of the three phases,
            // so computing it within each per-phase call is correct.
            double omega = 2.0 * Math.PI * frequencyVoltage;
            Func<double, double> sin_a = t => Math.Sin(omega * t + phaseVoltage);
            Func<double, double> sin_b = t => Math.Sin(omega * t + phaseVoltage - 2.0 / 3.0 * Math.PI);
            Func<double, double> sin_c = t => Math.Sin(omega * t + phaseVoltage - 4.0 / 3.0 * Math.PI);

            List<double> ret_voltage = new List<double> { };
            List<double> ret_time = new List<double> { };

            for (int i = 0; i < int.MaxValue; i++)
            {
                int _i = i % 200;
                double _t = i * 1.0 / this.SwitchingFrequency / 200.0;
                if (_t <= timeMax) ret_time.Add(_t);
                else break;

                // Zero-sequence injection (saddle waveform):
                //   u_zero = (max(ua, ub, uc) + min(ua, ub, uc)) / 2
                //   ua_new  = ua - u_zero
                // This is mathematically equivalent to traditional SVPWM.
                double ua = amplitudeVoltage * sin_a(_t);
                double ub = amplitudeVoltage * sin_b(_t);
                double uc = amplitudeVoltage * sin_c(_t);
                double u_zero = (Math.Max(ua, Math.Max(ub, uc)) + Math.Min(ua, Math.Min(ub, uc))) / 2.0;

                // Saddle-wave reference for the target phase
                double _u = ua - u_zero;

                double _driverUpper = driverUpper[_i];
                double _driverLower = driverLower[_i];

                if (_u > _driverUpper) ret_voltage.Add(this.DCLink / 2.0);
                else if (_u < _driverLower) ret_voltage.Add(-this.DCLink / 2.0);
                else ret_voltage.Add(0.0);
            }

            return new List<List<double>> { ret_time, ret_voltage };
        }
    }


    public class QuasiSVPWM2 : PWM, ICanOutputVoltage
    {
        public QuasiSVPWM2(double switchingFrequency, double dCLink) :
            base(switchingFrequency, dCLink)
        {
        }

        public List<List<double>> GetOutputVoltage(double amplitudeVoltage, double frequencyVoltage,
            double phaseVoltage, int periods)
        {
            double timeMax = periods / frequencyVoltage;
            List<double> driver = new List<double> { };
            List<double> time = new List<double> { };

            // Build single triangular carrier (-Vdc/2 → +Vdc/2), 200 points per switching period
            for (int i = 0; i < 50; i++)
            {
                driver.Add(-this.DCLink / 2.0 / 50.0 * i);
            }

            for (int i = 0; i < 100; i++)
            {
                driver.Add(-this.DCLink / 2.0 + this.DCLink / 100.0 * i);
            }

            for (int i = 0; i < 50; i++)
            {
                driver.Add(this.DCLink / 2.0 - this.DCLink / 2.0 / 50.0 * i);
            }

            for (int i = 0; i < 200; i++)
            {
                time.Add(1.0 / this.SwitchingFrequency / 200.0 * i);
            }
            driver.Add(0.0);

            // 3-phase sinusoidal references (computed internally for zero-sequence extraction).
            // The zero-sequence component is invariant to cyclic permutation of the three phases,
            // so computing it within each per-phase call is correct.
            double omega = 2.0 * Math.PI * frequencyVoltage;
            Func<double, double> sin_a = t => Math.Sin(omega * t + phaseVoltage);
            Func<double, double> sin_b = t => Math.Sin(omega * t + phaseVoltage - 2.0 / 3.0 * Math.PI);
            Func<double, double> sin_c = t => Math.Sin(omega * t + phaseVoltage - 4.0 / 3.0 * Math.PI);

            List<double> ret_voltage = new List<double> { };
            List<double> ret_time = new List<double> { };

            for (int i = 0; i < int.MaxValue; i++)
            {
                int _i = i % 200;
                double _t = i * 1.0 / this.SwitchingFrequency / 200.0;
                if (_t <= timeMax) ret_time.Add(_t);
                else break;

                // Zero-sequence injection (saddle waveform):
                //   u_zero = (max(ua, ub, uc) + min(ua, ub, uc)) / 2
                //   ua_new  = ua - u_zero
                // This is mathematically equivalent to traditional SVPWM.
                double ua = amplitudeVoltage * sin_a(_t);
                double ub = amplitudeVoltage * sin_b(_t);
                double uc = amplitudeVoltage * sin_c(_t);
                double u_zero = (Math.Max(ua, Math.Max(ub, uc)) + Math.Min(ua, Math.Min(ub, uc))) / 2.0;

                // Saddle-wave reference for the target phase
                double _u = ua - u_zero;

                double _driver = driver[_i];

                if (_u >= _driver) ret_voltage.Add(this.DCLink / 2.0);
                else ret_voltage.Add(-this.DCLink / 2.0);
            }

            return new List<List<double>> { ret_time, ret_voltage };
        }

    }



}

