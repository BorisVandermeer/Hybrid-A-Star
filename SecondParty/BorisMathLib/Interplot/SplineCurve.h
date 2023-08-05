/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A simple cubic spline path library without external 
 *      dependencies.
 * 
*********************************************************************/

#ifndef _INTERPLOATION_SPLINE_CURVE_H_
#define _INTERPLOATION_SPLINE_CURVE_H_

#include<cmath>
#include<vector>
#include<Interplot/Spline.h>

namespace Interplot
{
    class SplineCurve
    {
    public:
        typedef Spline::RefPoints  RefPoints;
        typedef Points::PosPoint2D Point;
        typedef Points::Pos2D      Pos2D;

        SplineCurve(){};
        ~SplineCurve(){};
        void setPoints(RefPoints refps);
        void setPoints(RefPoints refps,double FromHeading,double ToHeading);
        void rebuild(double stepsize);
        void setSplines(Spline _xs,Spline _ys);
        double getProjection(Point target,double max_s,double min_s,bool NewtownRefine = true, double gridsize = 0.5);
        double getDirectionalProjection(Pos2D target,double max_s,double min_s,bool NewtownRefine = true, double gridsize = 0.5);
        bool HasData() const  {return data_flag;}
        Point operator()(double s) const;
        double getHeading(double s){return std::atan2(ys_.getDeriv(1,s),xs_.getDeriv(1,s));}

    public:
        double max_s;

    private:
        Spline xs_,ys_;
        bool data_flag = false;
        double getProjectionByNewton(Point target,double s_hint, double s_max);
        double getDirectionalProjectionByNewton(Pos2D target,double s_hint, double s_max);

    };
    
} // namespace Interplot 

#endif