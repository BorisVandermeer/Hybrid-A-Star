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

#include<vector>
#include<Interplot/Spline.h>

namespace Interplot
{
    class SplineCurve
    {
        typedef Spline::RefPoints RefPoints;
    public:
        SplineCurve(){};
        ~SplineCurve(){};
        void setpoints(RefPoints refps);
        void setSplines(Spline _s_x,Spline _s_y);
        bool HasData() const  {return data_flag;}
        std::vector<double> operator()(double s) const;

    public:
        double max_s;

    private:
        Spline s_x,s_y;
        bool data_flag = false;

    };
    
} // namespace Interplot 

#endif