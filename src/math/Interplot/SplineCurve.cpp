/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A simple cubic spline path library without external 
 *      dependencies.
 * 
*********************************************************************/
#include<cmath>
#include<assert.h>
#include<Interplot/SplineCurve.h>

using namespace std;

namespace Interplot
{
    void SplineCurve::setpoints(RefPoints refps)
    {
        Spline::RefPoints points;
        vector<double> & s_list = points.x;
        vector<double> & x_list = refps.x;
        vector<double> & y_list = refps.y;
        assert(x_list.size()==y_list.size());
        size_t size = x_list.size();
        assert(size>2);
        s_list.resize(x_list.size());
        s_list[0] = 0;
        for(int i=1,maxi = s_list.size(); i<maxi ;i++)
        {
            s_list[i] = s_list[i-1]+sqrt((x_list[i]-x_list[i-1])*(x_list[i]-x_list[i-1])+(y_list[i]-y_list[i-1])*(y_list[i]-y_list[i-1]));
        }
        s_x.set_boundary(Spline::FirstOrderDer,(x_list[1]-x_list[0])/s_list[1],
                         Spline::FirstOrderDer,(x_list[size-1]-x_list[size-2])/(s_list[size-1]-s_list[size-2]),true);
        s_y.set_boundary(Spline::FirstOrderDer,(y_list[1]-y_list[0])/s_list[1],
                         Spline::FirstOrderDer,(y_list[size-1]-y_list[size-2])/(s_list[size-1]-s_list[size-2]),true);
        
        points.y = x_list;
        s_x.set_points(points);
        points.y = y_list;
        s_y.set_points(points);
        max_s = s_list.back();
        data_flag = true;
    }

    void SplineCurve::setSplines(Spline _s_x,Spline _s_y)
    {
        s_x = _s_x;
        s_y = _s_y;
    }

    std::vector<double> SplineCurve::operator()(double s) const
    {
        return {s_x(s),s_y(s)};
    }
} // namespace Interplot
