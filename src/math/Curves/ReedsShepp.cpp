/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Reference : 
 *      ompl/src/base/spaces
 * 
*********************************************************************/


#include<limits>
#include<math.h>
#include<assert.h>

#include<Curves/ReedsShepp.h>

namespace Curves
{
    const RSCurveSegmentType  RSCurveStateSpace::RSCurveStateSpace::RSCurveType[18][5] = {
        {RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP},         // 0
        {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP},        // 1
        {RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP},       // 2
        {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP},       // 3
        {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 4
        {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 5
        {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},    // 6
        {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},   // 7
        {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 8
        {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 9
        {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},   // 10
        {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},    // 11
        {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},     // 12
        {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},     // 13
        {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},      // 14
        {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},    // 15
        {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT},  // 16
        {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT}   // 17
    };
} // namespace Curves


namespace
{
    // The comments, variable names, etc. use the nomenclature from the Reeds & Shepp paper.
    const double pi = M_PI;
    const double twopi = 2. * pi;
#ifndef NDEBUG
    const double RS_EPS = 1e-6;
#endif
    const double ZERO = 10 * std::numeric_limits<double>::epsilon();

    inline static double mod2pi(double x)
    {
        double v = fmod(x, twopi);
        if (v < -pi)
            v += twopi;
        else if (v > pi)
            v -= twopi;
        return v;
    }
    inline void polar(double x, double y, double &r, double &theta)
    {
        r = sqrt(x * x + y * y);
        theta = atan2(y, x);
    }
    inline void tauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega)
    {
        double delta = mod2pi(u - v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1.;
        double t1 = atan2(eta * A - xi * B, xi * A + eta * B), t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
        tau = (t2 < 0) ? mod2pi(t1 + pi) : mod2pi(t1);
        omega = mod2pi(tau - u + v - phi);
    }

    // formula 8.1 in Reeds-Shepp paper
    inline bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v)
    {
        polar(x - sin(phi), y - 1. + cos(phi), u, t);
        if (t >= -ZERO)
        {
            v = mod2pi(phi - t);
            if (v >= -ZERO)
            {
                assert(fabs(u * cos(t) + sin(phi) - x) < RS_EPS);
                assert(fabs(u * sin(t) - cos(phi) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t + v - phi)) < RS_EPS);
                return true;
            }
        }
        return false;
    }
    // formula 8.2
    inline bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v)
    {
        double t1, u1;
        polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
        u1 = u1 * u1;
        if (u1 >= 4.)
        {
            double theta;
            u = sqrt(u1 - 4.);
            theta = atan2(2., u);
            t = mod2pi(t1 + theta);
            v = mod2pi(t - phi);
            assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < RS_EPS);
            assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
            return t >= -ZERO && v >= -ZERO;
        }
        return false;
    }
    void CSC(double x, double y, double phi, Curves::RSCurve &path)
    {
        double t, u, v, Lmin = path.TotalLength(), L;
        if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[14], t, u, v);
            Lmin = L;
        }
        if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[14], -t, -u, -v);
            Lmin = L;
        }
        if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[15], t, u, v);
            Lmin = L;
        }
        if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[15], -t, -u, -v);
            Lmin = L;
        }
        if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[12], t, u, v);
            Lmin = L;
        }
        if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[12], -t, -u, -v);
            Lmin = L;
        }
        if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[13], t, u, v);
            Lmin = L;
        }
        if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[13], -t, -u, -v);
    }
    // formula 8.3 / 8.4  *** TYPO IN PAPER ***
    inline bool LpRmL(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
        polar(xi, eta, u1, theta);
        if (u1 <= 4.)
        {
            u = -2. * asin(.25 * u1);
            t = mod2pi(theta + .5 * u + pi);
            v = mod2pi(phi - t + u);
            assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < RS_EPS);
            assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t - u + v - phi)) < RS_EPS);
            return t >= -ZERO && u <= ZERO;
        }
        return false;
    }
    void CCC(double x, double y, double phi, Curves::RSCurve &path)
    {
        double t, u, v, Lmin = path.TotalLength(), L;
        if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[0], t, u, v);
            Lmin = L;
        }
        if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[0], -t, -u, -v);
            Lmin = L;
        }
        if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[1], t, u, v);
            Lmin = L;
        }
        if (LpRmL(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[1], -t, -u, -v);
            Lmin = L;
        }

        // backwards
        double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
        if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[0], v, u, t);
            Lmin = L;
        }
        if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[0], -v, -u, -t);
            Lmin = L;
        }
        if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[1], v, u, t);
            Lmin = L;
        }
        if (LpRmL(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[1], -v, -u, -t);
    }
    // formula 8.7
    inline bool LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = .25 * (2. + sqrt(xi * xi + eta * eta));
        if (rho <= 1.)
        {
            u = acos(rho);
            tauOmega(u, -u, xi, eta, phi, t, v);
            assert(fabs(2 * (sin(t) - sin(t - u) + sin(t - 2 * u)) - sin(phi) - x) < RS_EPS);
            assert(fabs(2 * (-cos(t) + cos(t - u) - cos(t - 2 * u)) + cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t - 2 * u - v - phi)) < RS_EPS);
            return t >= -ZERO && v <= ZERO;
        }
        return false;
    }
    // formula 8.8
    inline bool LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = (20. - xi * xi - eta * eta) / 16.;
        if (rho >= 0 && rho <= 1)
        {
            u = -acos(rho);
            if (u >= -.5 * pi)
            {
                tauOmega(u, u, xi, eta, phi, t, v);
                assert(fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x) < RS_EPS);
                assert(fabs(-4 * cos(t) + 2 * cos(t - u) + cos(phi) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
                return t >= -ZERO && v >= -ZERO;
            }
        }
        return false;
    }
    void CCCC(double x, double y, double phi, Curves::RSCurve &path)
    {
        double t, u, v, Lmin = path.TotalLength(), L;
        if (LpRupLumRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[2], t, u, -u, v);
            Lmin = L;
        }
        if (LpRupLumRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[2], -t, -u, u, -v);
            Lmin = L;
        }
        if (LpRupLumRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[3], t, u, -u, v);
            Lmin = L;
        }
        if (LpRupLumRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[3], -t, -u, u, -v);
            Lmin = L;
        }

        if (LpRumLumRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[2], t, u, u, v);
            Lmin = L;
        }
        if (LpRumLumRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[2], -t, -u, -u, -v);
            Lmin = L;
        }
        if (LpRumLumRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[3], t, u, u, v);
            Lmin = L;
        }
        if (LpRumLumRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[3], -t, -u, -u, -v);
    }
    // formula 8.9
    inline bool LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
        polar(xi, eta, rho, theta);
        if (rho >= 2.)
        {
            double r = sqrt(rho * rho - 4.);
            u = 2. - r;
            t = mod2pi(theta + atan2(r, -2.));
            v = mod2pi(phi - .5 * pi - t);
            assert(fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x) < RS_EPS);
            assert(fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t + pi / 2 + v - phi)) < RS_EPS);
            return t >= -ZERO && u <= ZERO && v <= ZERO;
        }
        return false;
    }
    // formula 8.10
    inline bool LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
        polar(-eta, xi, rho, theta);
        if (rho >= 2.)
        {
            t = theta;
            u = 2. - rho;
            v = mod2pi(t + .5 * pi - phi);
            assert(fabs(2 * sin(t) - cos(t - v) - u * sin(t) - x) < RS_EPS);
            assert(fabs(-2 * cos(t) - sin(t - v) + u * cos(t) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t + pi / 2 - v - phi)) < RS_EPS);
            return t >= -ZERO && u <= ZERO && v <= ZERO;
        }
        return false;
    }
    void CCSC(double x, double y, double phi, Curves::RSCurve &path)
    {
        double t, u, v, Lmin = path.TotalLength() - .5 * pi, L;
        if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[4], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path =
                Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[4], -t, .5 * pi, -u, -v);
            Lmin = L;
        }
        if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[5], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LpRmSmLm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path =
                Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[5], -t, .5 * pi, -u, -v);
            Lmin = L;
        }

        if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[8], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path =
                Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[8], -t, .5 * pi, -u, -v);
            Lmin = L;
        }
        if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[9], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LpRmSmRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path =
                Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[9], -t, .5 * pi, -u, -v);
            Lmin = L;
        }

        // backwards
        double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
        if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[6], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path =
                Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[6], -v, -u, .5 * pi, -t);
            Lmin = L;
        }
        if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[7], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LpRmSmLm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path =
                Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[7], -v, -u, .5 * pi, -t);
            Lmin = L;
        }

        if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path =
                Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[10], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path =
                Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[10], -v, -u, .5 * pi, -t);
            Lmin = L;
        }
        if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path =
                Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[11], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LpRmSmRm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
            path =
                Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[11], -v, -u, .5 * pi, -t);
    }
    // formula 8.11 *** TYPO IN PAPER ***
    inline bool LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
        polar(xi, eta, rho, theta);
        if (rho >= 2.)
        {
            u = 4. - sqrt(rho * rho - 4.);
            if (u <= ZERO)
            {
                t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
                v = mod2pi(t - phi);
                assert(fabs(4 * sin(t) - 2 * cos(t) - u * sin(t) - sin(phi) - x) < RS_EPS);
                assert(fabs(-4 * cos(t) - 2 * sin(t) + u * cos(t) + cos(phi) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
                return t >= -ZERO && v >= -ZERO;
            }
        }
        return false;
    }
    void CCSCC(double x, double y, double phi, Curves::RSCurve &path)
    {
        double t, u, v, Lmin = path.TotalLength() - pi, L;
        if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[16], t, -.5 * pi, u,
                                                        -.5 * pi, v);
            Lmin = L;
        }
        if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[16], -t, .5 * pi, -u,
                                                        .5 * pi, -v);
            Lmin = L;
        }
        if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[17], t, -.5 * pi, u,
                                                        -.5 * pi, v);
            Lmin = L;
        }
        if (LpRmSLmRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
            path = Curves::RSCurve(Curves::RSCurveStateSpace::RSCurveType[17], -t, .5 * pi, -u,
                                                        .5 * pi, -v); 
    }

    Curves::RSCurve reedsShepp(double x, double y, double phi)
    {
        Curves::RSCurve path;
        CSC(x, y, phi, path);
        CCC(x, y, phi, path);
        CCCC(x, y, phi, path);
        CCSC(x, y, phi, path);
        CCSCC(x, y, phi, path);
        return path;
    }
}

namespace Curves
{
    RSCurve::RSCurve(const RSCurveSegmentType *type, double t, double u, double v, double w, double x, double _radius): type_(type){
        length[0] = t;
        length[1] = u;
        length[2] = v;
        length[3] = w;
        length[4] = x;
        totalLength_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
        radius = _radius;
    }

    RSCurve RSCurveStateSpace::RSCurveCalc(double x,double y,double phi) const{
        RSCurve ans =  ::reedsShepp(x / radius_, y / radius_, phi);
        ans.radius = radius_;
        return ans;
    }

    RSCurve RSCurveStateSpace::RSCurveCalc(State &from, State &to) const{
        State *s1 = &from, *s2 = &to;
        double x1 = s1->x, y1 = s1->y, th1 = s1->phi;
        double x2 = s2->x, y2 = s2->y, th2 = s2->phi;
        double dx = x2 - x1, dy = y2 - y1, c = cos(th1), s = sin(th1);
        double x = c * dx + s * dy, y = -s * dx + c * dy, phi = th2 - th1;
        return RSCurveCalc(x,y,phi);
    }


} // namespace Curves
