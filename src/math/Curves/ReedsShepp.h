/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Reference : 
 *      ompl/src/base/spaces
 * 
*********************************************************************/

#ifndef _CURVES_REEDS_SHEPP_CURVE_H_
#define _CURVES_REEDS_SHEPP_CURVE_H_

#include<cmath>
#include<limits>
#include<Points/PointsType.h>

namespace Curves{
    
    enum RSCurveSegmentType
    {
        RS_NOP = 0,
        RS_LEFT = 1,
        RS_STRAIGHT = 2,
        RS_RIGHT = 3
    };

    class RSCurve{
    public:
        RSCurve(){};
        RSCurve( const RSCurveSegmentType *type,
                double t = std::numeric_limits<double>::max(), double u = 0., double v = 0.,
                double w = 0., double x = 0.,double _radius=1.0);
        double TotalLength() const {return totalLength_*radius;}

        const RSCurveSegmentType *type_;
        double length[5];
        double radius = 1;

    private:
        double totalLength_ = std::numeric_limits<double>::max();
        
    };

    class RSCurveStateSpace{
    public:
        // RSCurveStateSpace() = delete;
        RSCurveStateSpace(double turningRadius = 1.0):radius_(turningRadius){};
        static const RSCurveSegmentType RSCurveType[18][5];
        typedef Points::Pos2D Pos2d;
        typedef Pos2d State;

        RSCurve RSCurveCalc(double x,double y,double phi) const;
        RSCurve RSCurveCalc(State & from, State & to) const;
        double getRadius() const {return radius_;};
        void setRadius(double turningRadius){radius_ = turningRadius;};
    protected:
        double radius_;
    };
    
} //namespace Curves

#endif