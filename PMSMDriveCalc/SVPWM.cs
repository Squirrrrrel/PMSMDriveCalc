using System;
using System.Collections.Generic;
using System.Numerics;



namespace PMSMDriveCalc
{
    public class SVPWM : PWM
    {
        public SVPWM(double switchingFrequency, double dCLink) : base(switchingFrequency, dCLink)
        {
        }
    }


    public struct SpaceVectorLocation
    {
        private int zone;
        private string subDivision;

        public int Zone { get => zone; set => zone = value; }
        public string SubDivision { get => subDivision; set => subDivision = value; }
    }




    public class SVPWM3 : SVPWM, ICanOutputVoltage
    {
        private static readonly List<int> n0 = new List<int> { -1, -1, -1 };
        private static readonly List<int> n1 = new List<int> { 0, -1, -1 };
        private static readonly List<int> n2 = new List<int> { 1, -1, -1 };
        private static readonly List<int> n3 = new List<int> { -1, 0, -1 };
        private static readonly List<int> n4 = new List<int> { 0, 0, -1 };
        private static readonly List<int> n5 = new List<int> { 1, 0, -1 };
        private static readonly List<int> n6 = new List<int> { -1, 1, -1 };
        private static readonly List<int> n7 = new List<int> { 0, 1, -1 };
        private static readonly List<int> n8 = new List<int> { 1, 1, -1 };
        private static readonly List<int> n9 = new List<int> { -1, -1, 0 };
        private static readonly List<int> n10 = new List<int> { 0, -1, 0 };
        private static readonly List<int> n11 = new List<int> { 1, -1, 0 };
        private static readonly List<int> n12 = new List<int> { -1, 0, 0 };
        private static readonly List<int> n13 = new List<int> { 0, 0, 0 };
        private static readonly List<int> n14 = new List<int> { 1, 0, 0 };
        private static readonly List<int> n15 = new List<int> { -1, 1, 0 };
        private static readonly List<int> n16 = new List<int> { 0, 1, 0 };
        private static readonly List<int> n17 = new List<int> { 1, 1, 0 };
        private static readonly List<int> n18 = new List<int> { -1, -1, 1 };
        private static readonly List<int> n19 = new List<int> { 0, -1, 1 };
        private static readonly List<int> n20 = new List<int> { 1, -1, 1 };
        private static readonly List<int> n21 = new List<int> { -1, 0, 1 };
        private static readonly List<int> n22 = new List<int> { 0, 0, 1 };
        private static readonly List<int> n23 = new List<int> { 1, 0, 1 };
        private static readonly List<int> n24 = new List<int> { -1, 1, 1 };
        private static readonly List<int> n25 = new List<int> { 0, 1, 1 };
        private static readonly List<int> n26 = new List<int> { 1, 1, 1 };

        public SVPWM3(double switchingFrequency, double dCLink) :
            base(switchingFrequency, dCLink)
        {
            this.SpaceVector3L = new SpaceVector3L(dCLink);
        }

        public SwitchingPatternSeries GetSwitchingPatternSeries(List<List<double>> voltages)
        {
            SwitchingPattern[] sps = new SwitchingPattern[voltages.Count];
            for (int i = 0; i < voltages.Count; i++)
            {
                List<double> voltage = voltages[i];
                var spaceVectorNow = this.SpaceVector3L.GetSpaceVector(voltage[0], voltage[1], voltage[2]);
                sps[i] = GetSwitchingPattern(spaceVectorNow);
            }
            SwitchingPatternSeries ret = new SwitchingPatternSeries();
            ret.VoltageSeries = voltages;
            ret.SwitchingPatterns = sps;
            return ret;
        }

        public List<List<double>> GetOutputVoltage(double amplitudeVoltage, double frequencyVoltage,
            double phaseVoltage, int periods)
        {
            double timeMax = periods / frequencyVoltage;

            Func<double, double> u_u0 = t => amplitudeVoltage * Math.Sin(2.0 *
                Math.PI * frequencyVoltage * t + phaseVoltage);
            Func<double, double> u_v0 = t => amplitudeVoltage * Math.Sin(2.0 *
                Math.PI * frequencyVoltage * t - 2.0 / 3.0 * Math.PI + phaseVoltage);
            Func<double, double> u_w0 = t => amplitudeVoltage * Math.Sin(2.0 *
                Math.PI * frequencyVoltage * t - 4.0 / 3.0 * Math.PI + phaseVoltage);

            int num = (int)Math.Ceiling(timeMax * this.SwitchingFrequency);
            List<double> uUVWRef;
            List<List<double>> uUVWRefList = new List<List<double>> { };
            for (int i = 0; i < num; i++)
            {
                double _t = 1 / this.SwitchingFrequency * i;
                uUVWRef = new List<double> { u_u0(_t), u_v0(_t), u_w0(_t) };
                uUVWRefList.Add(uUVWRef);
            }
            var patterns = GetSwitchingPatternSeries(uUVWRefList);
            List<double> tList = new List<double> { };
            List<double> uUList = new List<double> { };
            List<double> uVList = new List<double> { };
            List<double> uWList = new List<double> { };

            double t0 = 0.0;
            double tseg = 1.0 / SwitchingFrequency / 200.0;

            for (int i = 0; i < num; i++)
            {
                var dtList = patterns.SwitchingPatterns[i].TimeRatio;
                int num1 = (int)Math.Round((dtList[1] / SwitchingFrequency) / tseg);
                int num2 = (int)Math.Round((dtList[2] / SwitchingFrequency) / tseg);
                int num0 = (int)Math.Round((dtList[0] / SwitchingFrequency) / tseg);
                int num3 = Math.Max(0, (100 - num0 - num1 - num2) * 2);
                int num4 = num2;
                int num5 = num1;
                int num6 = num0;
                int[] num_timesegs = new int[] { num0, num1, num2, num3, num4, num5, num6 };
                for (int j = 0; j < 7; j++)
                {
                    int num_now = num_timesegs[j];
                    for (int k = 0; k < num_now; k++)
                    {
                        tList.Add(t0);
                        t0 += tseg;
                        uUList.Add(patterns.SwitchingPatterns[i].Voltages[j][0]);
                        uVList.Add(patterns.SwitchingPatterns[i].Voltages[j][1]);
                        uWList.Add(patterns.SwitchingPatterns[i].Voltages[j][2]);
                    }
                }
            }
            List<List<double>> ret = new List<List<double>> { tList,
            uUList, uVList, uWList};
            return ret;
        }

        public SwitchingPattern GetSwitchingPattern(Complex spaceVector)
        {
            SpaceVectorLocation _loc = this.SpaceVector3L.GetSubDivision(spaceVector);
            double _vm = spaceVector.Magnitude;
            double _m = _vm / (2.0 / 3.0 * this.SpaceVector3L.DCLink);

            string _loc_string = _loc.Zone.ToString() + "_" + _loc.SubDivision;
            List<List<int>> _vectors;
            switch (_loc_string)
            {
                case "1_A":
                    _vectors = new List<List<int>> { n13, n14, n17, n26 };
                    break;
                case "1_C":
                    _vectors = new List<List<int>> { n14, n5, n4, n1 };
                    break;
                case "1_B":
                    _vectors = new List<List<int>> { n14, n5, n2, n1 };
                    break;
                case "1_D":
                    _vectors = new List<List<int>> { n4, n5, n8, n17 };
                    break;


                case "2_A":
                    _vectors = new List<List<int>> { n13, n16, n17, n26 };
                    break;
                case "2_C":
                    _vectors = new List<List<int>> { n16, n7, n4, n3 };
                    break;
                case "2_B":
                    _vectors = new List<List<int>> { n4, n7, n8, n17 };
                    break;
                case "2_D":
                    _vectors = new List<List<int>> { n16, n7, n6, n3 };
                    break;


                case "3_A":
                    _vectors = new List<List<int>> { n13, n16, n25, n26 };
                    break;
                case "3_C":
                    _vectors = new List<List<int>> { n16, n15, n12, n3 };
                    break;
                case "3_B":
                    _vectors = new List<List<int>> { n16, n15, n6, n3 };
                    break;
                case "3_D":
                    _vectors = new List<List<int>> { n12, n15, n24, n25 };
                    break;


                case "4_A":
                    _vectors = new List<List<int>> { n13, n22, n25, n26 };
                    break;
                case "4_C":
                    _vectors = new List<List<int>> { n22, n21, n12, n9 };
                    break;
                case "4_B":
                    _vectors = new List<List<int>> { n12, n21, n24, n25 };
                    break;
                case "4_D":
                    _vectors = new List<List<int>> { n22, n21, n18, n9 };
                    break;


                case "5_A":
                    _vectors = new List<List<int>> { n13, n22, n23, n26 };
                    break;
                case "5_C":
                    _vectors = new List<List<int>> { n22, n19, n10, n9 };
                    break;
                case "5_B":
                    _vectors = new List<List<int>> { n22, n19, n18, n9 };
                    break;
                case "5_D":
                    _vectors = new List<List<int>> { n10, n19, n20, n23 };
                    break;


                case "6_A":
                    _vectors = new List<List<int>> { n13, n14, n23, n26 };
                    break;
                case "6_C":
                    _vectors = new List<List<int>> { n14, n11, n10, n1 };
                    break;
                case "6_B":
                    _vectors = new List<List<int>> { n10, n11, n20, n23 };
                    break;
                case "6_D":
                    _vectors = new List<List<int>> { n14, n11, n2, n1 };
                    break;

                default:
                    _vectors = new List<List<int>> { n0, n0, n0, n0 };
                    break;
            }

            Complex _vec1 = this.SpaceVector3L.GetSpaceVector(_vectors[1][0],
                _vectors[1][1], _vectors[1][2]) * this.SpaceVector3L.DCLink / 2.0;
            Complex _vec2 = this.SpaceVector3L.GetSpaceVector(_vectors[2][0],
                _vectors[2][1], _vectors[2][2]) * this.SpaceVector3L.DCLink / 2.0;
            Complex _vec3 = this.SpaceVector3L.GetSpaceVector(_vectors[0][0],
                _vectors[0][1], _vectors[0][2]) * this.SpaceVector3L.DCLink / 2.0;

            double _x1 = _vec1.Real;
            double _y1 = _vec1.Imaginary;
            double _x2 = _vec2.Real;
            double _y2 = _vec2.Imaginary;
            double _x3 = _vec3.Real;
            double _y3 = _vec3.Imaginary;

            double t1 = (_x2 * _y3 - _x2 * spaceVector.Imaginary - _x3 * _y2 +
                _x3 * spaceVector.Imaginary + spaceVector.Real * _y2 -
                spaceVector.Real * _y3) /
                (_x1 * _y2 - _x1 * _y3 - _x2 * _y1 + _x2 * _y3 + _x3 * _y1 - _x3 * _y2);

            double t2 = (-_x1 * _y3 + _x1 * spaceVector.Imaginary + _x3 * _y1 -
                _x3 * spaceVector.Imaginary - spaceVector.Real * _y1 +
                spaceVector.Real * _y3) /
                (_x1 * _y2 - _x1 * _y3 - _x2 * _y1 + _x2 * _y3 + _x3 * _y1 - _x3 * _y2);
            double t3 = 1.0 - t1 - t2;

            List<List<int>> _return_vectors = new List<List<int>>
            { _vectors[0], _vectors[1], _vectors[2], _vectors[0],
                _vectors[2], _vectors[1], _vectors[0]};

            List<double> _return_timeratio = new List<double>
            { t3 / 4.0, t1 / 2.0, t2 / 2.0, t3 / 2.0,
                t2 / 2.0, t1 / 2.0, t3 / 4.0};

            List<List<double>> _return_voltages = new List<List<double>>
            { _getVoltages(_return_vectors[0]), _getVoltages(_return_vectors[1]),
            _getVoltages(_return_vectors[2]), _getVoltages(_return_vectors[3]),
            _getVoltages(_return_vectors[4]), _getVoltages(_return_vectors[5]),
            _getVoltages(_return_vectors[6])};

            SwitchingPattern sp = new SwitchingPattern();
            sp.Vectors = _return_vectors;
            sp.TimeRatio = _return_timeratio;
            sp.Voltages = _return_voltages;
            sp.SpaceVectors = new List<Complex>
            {
                this.SpaceVector3L.DCLink / 2.0 * this.SpaceVector3L.GetSpaceVector(_return_vectors[0][0], _return_vectors[0][1], _return_vectors[0][2]),
                this.SpaceVector3L.DCLink / 2.0 * this.SpaceVector3L.GetSpaceVector(_return_vectors[1][0], _return_vectors[1][1], _return_vectors[1][2]),
                this.SpaceVector3L.DCLink / 2.0 * this.SpaceVector3L.GetSpaceVector(_return_vectors[2][0], _return_vectors[2][1], _return_vectors[2][2]),
                this.SpaceVector3L.DCLink / 2.0 * this.SpaceVector3L.GetSpaceVector(_return_vectors[3][0], _return_vectors[3][1], _return_vectors[3][2]),
                this.SpaceVector3L.DCLink / 2.0 * this.SpaceVector3L.GetSpaceVector(_return_vectors[4][0], _return_vectors[4][1], _return_vectors[4][2]),
                this.SpaceVector3L.DCLink / 2.0 * this.SpaceVector3L.GetSpaceVector(_return_vectors[5][0], _return_vectors[5][1], _return_vectors[5][2]),
                this.SpaceVector3L.DCLink / 2.0 * this.SpaceVector3L.GetSpaceVector(_return_vectors[6][0], _return_vectors[6][1], _return_vectors[6][2])
            };
            sp.ResultSpaceVector = sp.SpaceVectors[0] * sp.TimeRatio[0] +
                sp.SpaceVectors[1] * sp.TimeRatio[1] +
                sp.SpaceVectors[2] * sp.TimeRatio[2] +
                sp.SpaceVectors[3] * sp.TimeRatio[3] +
                sp.SpaceVectors[4] * sp.TimeRatio[4] +
                sp.SpaceVectors[5] * sp.TimeRatio[5] +
                sp.SpaceVectors[6] * sp.TimeRatio[6];
            sp.TargetSpaceVector = spaceVector;
            return sp;

        }

        public SpaceVector3L SpaceVector3L { get; private set; }

        private List<double> _getVoltages(List<int> vector)
        {
            List<double> ret = new List<double> { };
            for (int i = 0; i < 3; i++)
            {
                ret.Add(this.SpaceVector3L.DCLink / 2.0 * vector[i]);
            }
            return ret;
        }


    }


    public class SpaceVector3L
    {
        public double DCLink { get; private set; }

        public Point[] SubDivisionA { get; private set; }
        public Point[] SubDivisionB { get; private set; }
        public Point[] SubDivisionC { get; private set; }
        public Point[] SubDivisionD { get; private set; }

        public SpaceVector3L(double dCLink)
        {
            this.DCLink = dCLink;
            SubDivisionA = new Point[]
            {
                new Point(0.0, 0.0) * (2.0 / 3.0),
                new Point(DCLink / 2.0, 0.0) * (2.0 / 3.0),
                new Point(DCLink / 2.0 * Math.Cos(Math.PI / 3.0), DCLink / 2.0 * Math.Sin(Math.PI / 3.0)) * (2.0 / 3.0)
            };

            SubDivisionC = new Point[]
            {
                new Point(DCLink / 2.0, 0.0) * (2.0 / 3.0) +
                new Point(DCLink / 2.0 * Math.Cos(Math.PI / 3.0), DCLink / 2.0 * Math.Sin(Math.PI / 3.0)) * (2.0 / 3.0),
                new Point(DCLink / 2.0, 0.0) * (2.0 / 3.0),
                new Point(DCLink / 2.0 * Math.Cos(Math.PI / 3.0), DCLink / 2.0 * Math.Sin(Math.PI / 3.0)) * (2.0 / 3.0)
            };

            SubDivisionB = new Point[]
            {
                new Point(DCLink / 2.0, 0.0) * (2.0 / 3.0) +
                new Point(DCLink / 2.0 * Math.Cos(Math.PI / 3.0), DCLink / 2.0 * Math.Sin(Math.PI / 3.0)) * (2.0 / 3.0),
                new Point(DCLink / 2.0, 0.0) * (2.0 / 3.0),
                new Point(DCLink, 0.0) * (2.0 / 3.0)
            };

            SubDivisionD = new Point[]
            {
                new Point(DCLink / 2.0, 0.0) * (2.0 / 3.0) +
                new Point(DCLink / 2.0 * Math.Cos(Math.PI / 3.0), DCLink / 2.0 * Math.Sin(Math.PI / 3.0)) * (2.0 / 3.0),
                new Point(DCLink / 2.0 * Math.Cos(Math.PI / 3.0), DCLink / 2.0 * Math.Sin(Math.PI / 3.0)) * (2.0 / 3.0),
                new Point(DCLink * Math.Cos(Math.PI / 3.0), DCLink * Math.Sin(Math.PI / 3.0)) * (2.0 / 3.0)
            };
        }

        public Complex GetSpaceVector(double uu, double uv, double uw)
        {
            Complex j = new Complex(0.0, 1.0);
            Complex a = Complex.Pow(Math.E, j * 2.0 / 3.0 * Math.PI);
            Complex a2 = Complex.Pow(Math.E, j * 4.0 / 3.0 * Math.PI);
            Complex ret = uu + a * uv + a2 * uw;
            return ret * (2.0 / 3.0);
        }

        public static double GetAngleOfSpaceVector(Complex spaceVector)
        {
            return Math.Atan2(spaceVector.Imaginary, spaceVector.Real);
        }

        public SpaceVectorLocation GetSubDivision(Complex spaceVector)
        {
            SpaceVectorLocation _loc = new SpaceVectorLocation();

            double _ang = GetAngleOfSpaceVector(spaceVector);
            if (Math.Abs(_ang % (Math.PI / 3.0)) < 1e-5)
            {
                Complex j = new Complex(0.0, 1.0);
                Complex rot = Complex.Pow(Math.E, j * 0.001 / 180.0 * Math.PI);
                spaceVector *= rot;
            }
            _ang = GetAngleOfSpaceVector(spaceVector);

            int _zone = (int)Math.Floor(_ang / (Math.PI / 3.0));

            Point _target = new Point(spaceVector.Real, spaceVector.Imaginary);
            _target = _target.RotateByRad(new Point(0.0, 0.0), -_zone * Math.PI / 3.0);

            _zone += 1;
            if (_zone <= 0) _zone += 6;
            _loc.Zone = _zone;

            Point _distance = new Point(DCLink / 3.0, 0.0);
            Point _end1 = _target + _distance.RotateByDeg(new Point(0.0, 0.0), 45.0);
            Point _end2 = _target + _distance.RotateByDeg(new Point(0.0, 0.0), 165.0);
            Point _end3 = _target + _distance.RotateByDeg(new Point(0.0, 0.0), 285.0);

            CrossingDetector _cda = new CrossingDetector(SubDivisionA);
            CrossingDetector _cdb = new CrossingDetector(SubDivisionB);
            CrossingDetector _cdc = new CrossingDetector(SubDivisionC);
            CrossingDetector _cdd = new CrossingDetector(SubDivisionD);

            if (_cda.PolygonCrossingSingle(_target, _end1) &&
                _cda.PolygonCrossingSingle(_target, _end2) &&
                _cda.PolygonCrossingSingle(_target, _end3))
            {
                _loc.SubDivision = "A";
            }
            else if (_cdb.PolygonCrossingSingle(_target, _end1) &&
                _cdb.PolygonCrossingSingle(_target, _end2) &&
                _cdb.PolygonCrossingSingle(_target, _end3))
            {
                _loc.SubDivision = "B";
            }
            else if (_cdc.PolygonCrossingSingle(_target, _end1) &&
                _cdc.PolygonCrossingSingle(_target, _end2) &&
                _cdc.PolygonCrossingSingle(_target, _end3))
            {
                _loc.SubDivision = "C";
            }
            else if (_cdd.PolygonCrossingSingle(_target, _end1) &&
                _cdd.PolygonCrossingSingle(_target, _end2) &&
                _cdd.PolygonCrossingSingle(_target, _end3))
            {
                _loc.SubDivision = "D";
            }
            else _loc.SubDivision = " ";
            return _loc;

        }
    }

    public struct SwitchingPattern
    {
        private List<List<int>> vectors;
        private List<double> timeRatio;
        private List<List<double>> voltages;
        private List<Complex> spaceVectors;
        private Complex resultSpaceVector;
        private Complex targetSpaceVector;

        public List<List<int>> Vectors { get => vectors; set => vectors = value; }
        public List<double> TimeRatio { get => timeRatio; set => timeRatio = value; }
        public List<List<double>> Voltages { get => voltages; set => voltages = value; }
        public List<Complex> SpaceVectors { get => spaceVectors; set => spaceVectors = value; }
        public Complex ResultSpaceVector { get => resultSpaceVector; set => resultSpaceVector = value; }
        public Complex TargetSpaceVector { get => targetSpaceVector; set => targetSpaceVector = value; }
    }

    public class SwitchingPatternSeries
    {
        public SwitchingPattern[] SwitchingPatterns;
        public List<List<double>> VoltageSeries;
    }









    public class SVPWM2 : SVPWM, ICanOutputVoltage
    {
        private static readonly List<int> n0 = new List<int> { -1, -1, -1 };
        private static readonly List<int> n1 = new List<int> { 1, -1, -1 };
        private static readonly List<int> n2 = new List<int> { 1, 1, -1 };
        private static readonly List<int> n3 = new List<int> { -1, 1, -1 };
        private static readonly List<int> n4 = new List<int> { -1, 1, 1 };
        private static readonly List<int> n5 = new List<int> { -1, -1, 1 };
        private static readonly List<int> n6 = new List<int> { 1, -1, 1 };
        private static readonly List<int> n7 = new List<int> { 1, 1, 1 };

        public SVPWM2(double switchingFrequency, double dCLink) :
            base(switchingFrequency, dCLink)
        {
            this.SpaceVector2L = new SpaceVector2L(dCLink);
        }

        public SwitchingPatternSeries GetSwitchingPatternSeries(List<List<double>> voltages)
        {
            SwitchingPattern[] sps = new SwitchingPattern[voltages.Count];
            for (int i = 0; i < voltages.Count; i++)
            {
                List<double> voltage = voltages[i];
                var spaceVectorNow = this.SpaceVector2L.GetSpaceVector(voltage[0], voltage[1], voltage[2]);
                sps[i] = GetSwitchingPattern(spaceVectorNow);
            }
            SwitchingPatternSeries ret = new SwitchingPatternSeries();
            ret.VoltageSeries = voltages;
            ret.SwitchingPatterns = sps;
            return ret;
        }

        public List<List<double>> GetOutputVoltage(double amplitudeVoltage, double frequencyVoltage,
            double phaseVoltage, int periods)
        {
            double timeMax = periods / frequencyVoltage;

            Func<double, double> u_u0 = t => amplitudeVoltage * Math.Sin(2.0 *
                Math.PI * frequencyVoltage * t + phaseVoltage);
            Func<double, double> u_v0 = t => amplitudeVoltage * Math.Sin(2.0 *
                Math.PI * frequencyVoltage * t - 2.0 / 3.0 * Math.PI + phaseVoltage);
            Func<double, double> u_w0 = t => amplitudeVoltage * Math.Sin(2.0 *
                Math.PI * frequencyVoltage * t - 4.0 / 3.0 * Math.PI + phaseVoltage);

            int num = (int)Math.Ceiling(timeMax * this.SwitchingFrequency);
            List<List<double>> uUVWRefList = new List<List<double>>(num);
            for (int i = 0; i < num; i++)
            {
                double _t = i / this.SwitchingFrequency;
                uUVWRefList.Add(new List<double> { u_u0(_t), u_v0(_t), u_w0(_t) });
            }
            var patterns = GetSwitchingPatternSeries(uUVWRefList);

            int pointsPerPeriod = 200;
            int totalExpanded = num * pointsPerPeriod;
            double dt = 1.0 / this.SwitchingFrequency / pointsPerPeriod;
            List<double> tListExpanded = new List<double>(totalExpanded);
            List<double> uUListExpanded = new List<double>(totalExpanded);
            List<double> uVListExpanded = new List<double>(totalExpanded);
            List<double> uWListExpanded = new List<double>(totalExpanded);

            for (int i = 0; i < num; i++)
            {
                var dtList = patterns.SwitchingPatterns[i].TimeRatio;
                var voltages = patterns.SwitchingPatterns[i].Voltages;
                double tBase = i / this.SwitchingFrequency;
                double cumTimeRatio = 0.0;
                int j = 0;
                for (int k = 0; k < pointsPerPeriod; k++)
                {
                    double tau = k / (double)pointsPerPeriod;
                    while (j < 6 && tau >= cumTimeRatio + dtList[j] - 1e-15)
                    {
                        cumTimeRatio += dtList[j];
                        j++;
                    }
                    tListExpanded.Add(tBase + tau / this.SwitchingFrequency);
                    uUListExpanded.Add(voltages[j][0]);
                    uVListExpanded.Add(voltages[j][1]);
                    uWListExpanded.Add(voltages[j][2]);
                }
            }

            return new List<List<double>> { tListExpanded,
                uUListExpanded, uVListExpanded, uWListExpanded };
        }

        public SwitchingPattern GetSwitchingPattern(Complex spaceVector)
        {
            SpaceVectorLocation _loc = this.SpaceVector2L.GetSubDivision(spaceVector);
            double _vm = spaceVector.Magnitude;
            double _m = _vm / (2.0 / 3.0 * this.SpaceVector2L.DCLink);

            string _loc_string = _loc.Zone.ToString() + "_" + _loc.SubDivision;
            List<List<int>> _vectors;
            switch (_loc_string)
            {
                case "1_A":
                    _vectors = new List<List<int>> { n0, n1, n2, n7 };
                    break;

                case "2_A":
                    _vectors = new List<List<int>> { n0, n3, n2, n7 };
                    break;

                case "3_A":
                    _vectors = new List<List<int>> { n0, n3, n4, n7 };
                    break;

                case "4_A":
                    _vectors = new List<List<int>> { n0, n5, n4, n7 };
                    break;

                case "5_A":
                    _vectors = new List<List<int>> { n0, n5, n6, n7 };
                    break;

                case "6_A":
                    _vectors = new List<List<int>> { n0, n1, n6, n7 };
                    break;

                default:
                    _vectors = new List<List<int>> { n0, n0, n0, n0 };
                    break;
            }

            Complex _vec1 = this.SpaceVector2L.GetSpaceVector(_vectors[1][0],
                _vectors[1][1], _vectors[1][2]) * this.SpaceVector2L.DCLink / 2.0;
            Complex _vec2 = this.SpaceVector2L.GetSpaceVector(_vectors[2][0],
                _vectors[2][1], _vectors[2][2]) * this.SpaceVector2L.DCLink / 2.0;
            Complex _vec3 = this.SpaceVector2L.GetSpaceVector(_vectors[0][0],
                _vectors[0][1], _vectors[0][2]) * this.SpaceVector2L.DCLink / 2.0;

            double _x1 = _vec1.Real;
            double _y1 = _vec1.Imaginary;
            double _x2 = _vec2.Real;
            double _y2 = _vec2.Imaginary;
            double _x3 = _vec3.Real;
            double _y3 = _vec3.Imaginary;

            double t1 = (_x2 * _y3 - _x2 * spaceVector.Imaginary - _x3 * _y2 +
                _x3 * spaceVector.Imaginary + spaceVector.Real * _y2 -
                spaceVector.Real * _y3) /
                (_x1 * _y2 - _x1 * _y3 - _x2 * _y1 + _x2 * _y3 + _x3 * _y1 - _x3 * _y2);

            double t2 = (-_x1 * _y3 + _x1 * spaceVector.Imaginary + _x3 * _y1 -
                _x3 * spaceVector.Imaginary - spaceVector.Real * _y1 +
                spaceVector.Real * _y3) /
                (_x1 * _y2 - _x1 * _y3 - _x2 * _y1 + _x2 * _y3 + _x3 * _y1 - _x3 * _y2);
            double t3 = 1.0 - t1 - t2;

            List<List<int>> _return_vectors = new List<List<int>>
            { _vectors[0], _vectors[1], _vectors[2], _vectors[3],
                _vectors[2], _vectors[1], _vectors[0]};

            List<double> _return_timeratio = new List<double>
            { t3 / 4.0, t1 / 2.0, t2 / 2.0, t3 / 2.0,
                t2 / 2.0, t1 / 2.0, t3 / 4.0};

            List<List<double>> _return_voltages = new List<List<double>>
            { _getVoltages(_return_vectors[0]), _getVoltages(_return_vectors[1]),
            _getVoltages(_return_vectors[2]), _getVoltages(_return_vectors[3]),
            _getVoltages(_return_vectors[4]), _getVoltages(_return_vectors[5]),
            _getVoltages(_return_vectors[6])};

            SwitchingPattern sp = new SwitchingPattern();
            sp.Vectors = _return_vectors;
            sp.TimeRatio = _return_timeratio;
            sp.Voltages = _return_voltages;
            sp.SpaceVectors = new List<Complex>
            {
                this.SpaceVector2L.DCLink / 2.0 * this.SpaceVector2L.GetSpaceVector(_return_vectors[0][0], _return_vectors[0][1], _return_vectors[0][2]),
                this.SpaceVector2L.DCLink / 2.0 * this.SpaceVector2L.GetSpaceVector(_return_vectors[1][0], _return_vectors[1][1], _return_vectors[1][2]),
                this.SpaceVector2L.DCLink / 2.0 * this.SpaceVector2L.GetSpaceVector(_return_vectors[2][0], _return_vectors[2][1], _return_vectors[2][2]),
                this.SpaceVector2L.DCLink / 2.0 * this.SpaceVector2L.GetSpaceVector(_return_vectors[3][0], _return_vectors[3][1], _return_vectors[3][2]),
                this.SpaceVector2L.DCLink / 2.0 * this.SpaceVector2L.GetSpaceVector(_return_vectors[4][0], _return_vectors[4][1], _return_vectors[4][2]),
                this.SpaceVector2L.DCLink / 2.0 * this.SpaceVector2L.GetSpaceVector(_return_vectors[5][0], _return_vectors[5][1], _return_vectors[5][2]),
                this.SpaceVector2L.DCLink / 2.0 * this.SpaceVector2L.GetSpaceVector(_return_vectors[6][0], _return_vectors[6][1], _return_vectors[6][2])
            };
            sp.ResultSpaceVector = sp.SpaceVectors[0] * sp.TimeRatio[0] +
                sp.SpaceVectors[1] * sp.TimeRatio[1] +
                sp.SpaceVectors[2] * sp.TimeRatio[2] +
                sp.SpaceVectors[3] * sp.TimeRatio[3] +
                sp.SpaceVectors[4] * sp.TimeRatio[4] +
                sp.SpaceVectors[5] * sp.TimeRatio[5] +
                sp.SpaceVectors[6] * sp.TimeRatio[6];
            sp.TargetSpaceVector = spaceVector;
            return sp;

        }

        public SpaceVector2L SpaceVector2L { get; private set; }

        private List<double> _getVoltages(List<int> vector)
        {
            List<double> ret = new List<double> { };
            for (int i = 0; i < 3; i++)
            {
                ret.Add(this.SpaceVector2L.DCLink / 2.0 * vector[i]);
            }
            return ret;
        }


    }


    public class SpaceVector2L
    {
        public double DCLink { get; private set; }

        public Point[] SubDivisionA { get; private set; }

        public SpaceVector2L(double dCLink)
        {
            this.DCLink = dCLink;
            SubDivisionA = new Point[]
            {
                new Point(0.0, 0.0) * (2.0 / 3.0),
                new Point(DCLink, 0.0) * (2.0 / 3.0),
                new Point(DCLink * Math.Cos(Math.PI / 3.0), DCLink * Math.Sin(Math.PI / 3.0)) * (2.0 / 3.0)
            };

        }

        public Complex GetSpaceVector(double uu, double uv, double uw)
        {
            Complex j = new Complex(0.0, 1.0);
            Complex a = Complex.Pow(Math.E, j * 2.0 / 3.0 * Math.PI);
            Complex a2 = Complex.Pow(Math.E, j * 4.0 / 3.0 * Math.PI);
            Complex ret = uu + a * uv + a2 * uw;
            return ret * (2.0 / 3.0);
        }

        public static double GetAngleOfSpaceVector(Complex spaceVector)
        {
            return Math.Atan2(spaceVector.Imaginary, spaceVector.Real);
        }

        public SpaceVectorLocation GetSubDivision(Complex spaceVector)
        {
            SpaceVectorLocation _loc = new SpaceVectorLocation();

            double _ang = GetAngleOfSpaceVector(spaceVector);
            if (Math.Abs(_ang % (Math.PI / 3.0)) < 1e-5)
            {
                Complex j = new Complex(0.0, 1.0);
                Complex rot = Complex.Pow(Math.E, j * 0.001 / 180.0 * Math.PI);
                spaceVector *= rot;
            }
            _ang = GetAngleOfSpaceVector(spaceVector);

            int _zone = (int)Math.Floor(_ang / (Math.PI / 3.0));

            Point _target = new Point(spaceVector.Real, spaceVector.Imaginary);
            _target = _target.RotateByRad(new Point(0.0, 0.0), -_zone * Math.PI / 3.0);

            _zone += 1;
            if (_zone <= 0) _zone += 6;
            _loc.Zone = _zone;

            Point _distance = new Point(DCLink, 0.0) * (2.0 / 3.0);
            Point _end1 = _target + _distance.RotateByDeg(new Point(0.0, 0.0), 45.0);
            Point _end2 = _target + _distance.RotateByDeg(new Point(0.0, 0.0), 165.0);
            Point _end3 = _target + _distance.RotateByDeg(new Point(0.0, 0.0), 285.0);

            CrossingDetector _cda = new CrossingDetector(SubDivisionA);

            if (_cda.PolygonCrossingSingle(_target, _end1) &&
                _cda.PolygonCrossingSingle(_target, _end2) &&
                _cda.PolygonCrossingSingle(_target, _end3))
            {
                _loc.SubDivision = "A";
            }
            else _loc.SubDivision = " ";
            return _loc;

        }
    }



}