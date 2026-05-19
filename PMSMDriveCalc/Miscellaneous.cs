using System;
using Complex2 = Meta.Numerics.Complex;
using Complex = System.Numerics.Complex;
using Meta.Numerics.SignalProcessing;
using System.Numerics;
using System.Collections.Generic;

namespace PMSMDriveCalc
{

    public struct FFTContainer
    {
        private List<double> phase;
        private List<int> order;
        private List<double> amplitude;

        public List<double> Phase { get => phase; set => phase = value; }
        public List<int> Order { get => order; set => order = value; }
        public List<double> Amplitude { get => amplitude; set => amplitude = value; }

    }


    public class Point
    {
        public double X { get; private set; }
        public double Y { get; private set; }
        public Point(double x, double y)
        {
            this.X = x;
            this.Y = y;
        }

        public static Point operator +(Point pa, Point pb)
        {
            return new Point(pa.X + pb.X, pa.Y + pb.Y);
        }
        public static Point operator -(Point pa, Point pb)
        {
            return new Point(pa.X - pb.X, pa.Y - pb.Y);
        }
        //public static Point operator /(Point pa, double b)
        //{
        //    return new Point(pa.X / b, pa.Y / b);
        //}
        public static double operator *(Point pa, Point pb)
        {
            return pa.X * pb.X + pa.Y * pb.Y;
        }
        public static Point operator *(Point pa, double b)
        {
            return new Point(pa.X * b, pa.Y * b);
        }
        public Point RotateByRad(Point pbase, double angle)
        {
            var p0 = new Point(this.X - pbase.X, this.Y - pbase.Y);
            var p1 = new Point(p0.X * Math.Cos(angle) - p0.Y * Math.Sin(angle),
                p0.Y * Math.Cos(angle) + p0.X * Math.Sin(angle));
            return p1 + pbase;
        }
        public Point RotateByDeg(Point pbase, double angle)
        {
            return RotateByRad(pbase, angle / 180.0 * Math.PI);
        }
        //public Point MirrorX()
        //{ return new Point(this.X, -this.Y); }
        //public Point MirrorY()
        //{ return new Point(-this.X, this.Y); }
        //public double[] ConvertToArray()
        //{
        //    return new double[] { this.X, this.Y };
        //}
        //public double GetDistance(Point p2)
        //{
        //    return Math.Sqrt(Math.Pow(this.X - p2.X, 2.0) + Math.Pow(this.Y - p2.Y, 2.0));
        //}
        //public static double GetAngleRad(Point pbase, Point p1, Point p2)
        //{
        //    var p1_ = p1 - pbase;
        //    var p2_ = p2 - pbase;
        //    double ang1 = Math.Atan2(p1_.Y, p1_.X);
        //    double ang2 = Math.Atan2(p2_.Y, p2_.X);
        //    return ang2 - ang1;
        //}
        //public static double GetAngleDeg(Point pbase, Point p1, Point p2)
        //{
        //    return GetAngleRad(pbase, p1, p2) / Math.PI * 180.0;
        //}
    }

    public class CrossingDetector
    {
        public Point[] Polygon { get; private set; }
        public CrossingDetector(Point[] polygon)
        {
            Polygon = polygon;
        }

        public static Point CrossingPoint(Point p1, Point p2, Point p3, Point p4)
        {
            double x1 = p1.X;
            double y1 = p1.Y;
            double x2 = p2.X;
            double y2 = p2.Y;
            double x3 = p3.X;
            double y3 = p3.Y;
            double x4 = p4.X;
            double y4 = p4.Y;

            double x_res, y_res;
            double denominator = x1 * y3 - x1 * y4 - x2 * y3 + x2 * y4 - x3 * y1 + x3 * y2 + x4 * y1 - x4 * y2;
            if (Math.Abs(denominator) < 1e-12) return null;
            x_res = (x1 * x3 * y2 - x1 * x3 * y4 - x1 * x4 * y2 + x1 * x4 * y3 -
                x2 * x3 * y1 + x2 * x3 * y4 + x2 * x4 * y1 - x2 * x4 * y3) / denominator;
            y_res = (x1 * y2 * y3 - x1 * y2 * y4 - x2 * y1 * y3 + x2 * y1 * y4 -
                x3 * y1 * y4 + x3 * y2 * y4 + x4 * y1 * y3 - x4 * y2 * y3) / denominator;
            return new Point(x_res, y_res);
        }

        public static int Crossing(Point p1, Point p2, Point p3, Point p4)
        {
            Point pm = CrossingPoint(p1, p2, p3, p4);
            if (pm == null) { return 0; }
            double r12 = (p1 - pm) * (p2 - pm);
            double r34 = (p3 - pm) * (p4 - pm);

            if ((r12 <= 0) && (r34 <= 0)) { return 1; }
            else { return 0; }
        }

        public bool PolygonCrossingSingle(Point p, Point pref)
        {
            int len = Polygon.Length;
            int[] crossed = new int[len];
            Point p1, p2;
            for (int i = 0; i < len; i++)
            {
                p1 = Polygon[i];
                if (i != len - 1) { p2 = Polygon[i + 1]; }
                else { p2 = Polygon[0]; }
                var res = Crossing(p1, p2, p, pref);
                crossed[i] = res;
            }
            int sum = 0;
            foreach (int cross in crossed) { sum += cross; }
            bool ret = (sum % 2 == 1) ? true : false;
            return ret;
        }
    }




    public static class ListOperations
    {
        public static List<double> Plus(List<double> a, List<double> b)
        {
            int lena = a.Count;
            int lenb = b.Count;
            int len = (lena > lenb) ? lenb : lena;
            List<double> ret = new List<double> { };
            for (int i = 0; i < len; i++)
            {
                ret.Add(a[i] + b[i]);
            }
            return ret;
        }

        public static List<double> Minus(List<double> a, List<double> b)
        {
            int lena = a.Count;
            int lenb = b.Count;
            int len = (lena > lenb) ? lenb : lena;
            List<double> ret = new List<double> { };
            for (int i = 0; i < len; i++)
            {
                ret.Add(a[i] - b[i]);
            }
            return ret;
        }

        public static List<int> GetPreviousIndex(List<double> x_old, List<double> x_new)
        {
            int len = x_old.Count;
            int len_new = x_new.Count;
            int[] ret = new int[len_new];
            for (int j = 0; j < len_new; j++)
            {
                double x_now = x_new[j];
                int i;
                int found = 0;
                for (i = 0; i < len - 1; i++)
                {
                    if ( x_now <= x_old[i + 1])
                    {
                        ret[j] = i;
                        found = 1;
                        break;
                    }
                    found = 0;
                }
                if (found == 0) ret[j] = len - 1;
            }
            return new List<int> (ret);

        }
    }


    public static class FFTOperations
    {
        public static FFTContainer GetFFT(double[] source)
        {
            int len = source.Length;
            FourierTransformer ft = new FourierTransformer(len);
            Complex2[] _source = new Complex2[len];
            for (int i = 0; i < len; i++)
            {
                _source[i] = new Complex2(source[i], 0.0);
            }
            Complex2[] res = ft.Transform(_source);

            int num = (int)Math.Floor(source.Length / 2.0);
            double[] _amplitude = new double[num];
            double[] _phase = new double[num];
            int[] _order = new int[num];

            for (int i = 0; i < num; i++)
            {
                Complex2 value = res[i] / len * 2.0;
                _amplitude[i] = Math.Sqrt(Math.Pow(value.Re, 2.0) + Math.Pow(value.Im, 2.0));
                _phase[i] = Math.Atan2(value.Im, value.Re) / Math.PI * 180.0;
                _order[i] = i;
            }
            _amplitude[0] = _amplitude[0] / 2.0;
            FFTContainer ret = new FFTContainer();
            ret.Amplitude = new List<double>(_amplitude);
            ret.Phase = new List<double>(_phase);
            ret.Order = new List<int>(_order);
            return ret;
        }
    }

}
