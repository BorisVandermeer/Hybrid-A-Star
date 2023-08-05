/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A simple cubic spline path library without external 
 *      dependencies.
 * 
*********************************************************************/
#include<cmath>
#include<cfloat>
#include<assert.h>
#include<Interplot/SplineCurve.h>

#define NEWTOWN_ERROR (1e-5)
#define DIVIDE_ERR    (1e-5)

using namespace std;

namespace Interplot
{
    static inline double distanceSq(SplineCurve::Point p1, SplineCurve::Point p2){
        return (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y);
    }

    static inline double distance(SplineCurve::Point p1, SplineCurve::Point p2){
        return std::sqrt(distanceSq(p1,p2));
    }

    void SplineCurve::setPoints(RefPoints refps){
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
        xs_.set_boundary(Spline::FirstOrderDer,(x_list[1]-x_list[0])/s_list[1],
                         Spline::FirstOrderDer,(x_list[size-1]-x_list[size-2])/(s_list[size-1]-s_list[size-2]),true);
        ys_.set_boundary(Spline::FirstOrderDer,(y_list[1]-y_list[0])/s_list[1],
                         Spline::FirstOrderDer,(y_list[size-1]-y_list[size-2])/(s_list[size-1]-s_list[size-2]),true);
        
        points.y = x_list;
        xs_.set_points(points);
        points.y = y_list;
        ys_.set_points(points);
        max_s = s_list.back();
        data_flag = true;
    }

    void SplineCurve::setPoints(SplineCurve::RefPoints refps,double FromHeading,double ToHeading){
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
        xs_.set_boundary(Spline::FirstOrderDer,cos(FromHeading),
                         Spline::FirstOrderDer,cos(ToHeading),true);
        ys_.set_boundary(Spline::FirstOrderDer,sin(FromHeading),
                         Spline::FirstOrderDer,sin(ToHeading),true);
        points.y = x_list;
        xs_.set_points(points);
        points.y = y_list;
        ys_.set_points(points);
        max_s = s_list.back();
        data_flag = true;
    }

    void SplineCurve::rebuild(double stepsize){
        double FXDeriv = xs_.getDeriv(1,0);
        double EXDeriv = xs_.getDeriv(1,max_s);
        double FYDeriv = ys_.getDeriv(1,0);
        double EYDeriv = ys_.getDeriv(1,max_s);
        Spline::RefPoints points;
        vector<double> x;
        vector<double> y;
        vector<double> s;
        int size = max_s / stepsize;
        int vecsize = size;
        if((max_s - size*stepsize)>(stepsize*0.1)) vecsize++;
        x.resize(vecsize);y.resize(vecsize);s.resize(vecsize);
        double cur_s = 0;
        s[0] = 0; x[0] = xs_(0); y[0] = ys_(0);
        for(int i = 1;i<size;i++){
            cur_s+=stepsize;
            x[i] = xs_(cur_s); y[i] = ys_(cur_s);
            s[i] = s[i-1]+distance(Point(x[i],y[i]),Point(x[i-1],y[i-1]));
        }
        int i = vecsize-1;
        x[i] = xs_(max_s); y[i] = ys_(max_s);
        s[i] = s[i-1]+distance(Point(x[i],y[i]),Point(x[i-1],y[i-1]));
        xs_.clear();ys_.clear();

        xs_.set_boundary(Spline::FirstOrderDer,FXDeriv,
                         Spline::FirstOrderDer,EXDeriv,true);
        ys_.set_boundary(Spline::FirstOrderDer,FYDeriv,
                         Spline::FirstOrderDer,EYDeriv,true);
        
        points.y = x;
        points.x = s;
        xs_.set_points(points);
        points.y = y;
        ys_.set_points(points);
        max_s = s.back();
    }

    void SplineCurve::setSplines(Spline _xs,Spline _ys){
        xs_ = _xs;
        ys_ = _ys;
    }

    SplineCurve::Point SplineCurve::operator()(double s) const
    {
        return Point(xs_(s),ys_(s));
    }

    double SplineCurve::getProjection(Point target,double max_s,double min_s,bool NewtownRefine, double grid){
        if(max_s<min_s){
            return min_s;
        }
        double tmp_s = min_s, min_dis_s = min_s;
        auto min_dissq = DBL_MAX;
        while(tmp_s<max_s){
            auto tmpp = this->operator()(tmp_s);
            double tmp_dissq = distanceSq(target,tmpp);
            if(tmp_dissq<min_dissq){
                min_dissq = tmp_dissq;
                min_dis_s = tmp_s;
            }
            tmp_s += grid;
        }
        if(NewtownRefine) return getProjectionByNewton(target,min_dis_s,max_s);
        return min_dis_s;
    }

    double SplineCurve::getProjectionByNewton(Point target,double s_hint, double s_max){
        s_hint = std::min(s_hint,s_max);
        double s_cur  = s_hint;
        double s_prev = s_hint;
        for(int i=0;i<20;i++){
            double x = xs_(s_cur);
            double y = ys_(s_cur);
            double dx = xs_.getDeriv(1, s_cur);
            double dy = ys_.getDeriv(1, s_cur);
            double ddx = xs_.getDeriv(2, s_cur);
            double ddy = ys_.getDeriv(2, s_cur);
            double j = (x - target.x) * dx + (y - target.y) * dy;
            double h = dx * dx + (x - target.x) * ddx + dy * dy + (y - target.y) * ddy;
            s_cur -= j / h;
            if (fabs(s_cur - s_prev) < NEWTOWN_ERROR) break;
            s_prev = s_cur;
        }
        return std::min(s_cur, s_max);
    }

    double SplineCurve::getDirectionalProjection(Pos2D target,double max_s,double min_s,bool NewtownRefine, double gridsize){
        if (max_s <= min_s) return min_s;
        static const double grid = 2.0;
        double tmp_s = min_s, min_dot_value_s = min_s;
        double v1 = sin(target.phi);
        double v2 = -cos(target.phi);
        auto min_dot_value = DBL_MAX;
        while (tmp_s <= max_s) {
            Point state_on_spline{xs_(tmp_s), ys_(tmp_s)};
            double tmp_dot_value = fabs(v1 * (state_on_spline.x - target.x) + v2 * (state_on_spline.y - target.y));
            if (tmp_dot_value < min_dot_value) {
                tmp_dot_value = min_dot_value;
                min_dot_value_s = tmp_s;
            }
            tmp_s += grid;
        }
        // Newton's method
        if(NewtownRefine) return getDirectionalProjectionByNewton(target, min_dot_value_s, max_s);
        return min_dot_value_s;
    }
    
    double SplineCurve::getDirectionalProjectionByNewton(Pos2D target,double s_hint, double s_max){
        s_hint = std::min(s_hint, max_s);
        double cur_s = s_hint;
        double prev_s = s_hint;
        double v1 = sin(target.phi);
        double v2 = -cos(target.phi);
        for (int i = 0; i < 20; ++i) {
            double x = xs_(cur_s);
            double y = ys_(cur_s);
            double dx = xs_.getDeriv(1, cur_s);
            double dy = ys_.getDeriv(1, cur_s);
            double ddx = xs_.getDeriv(2, cur_s);
            double ddy = ys_.getDeriv(2, cur_s);
            // Ignore coeff 2 in J and H.
            double p1 = v1 * (x - target.x) + v2 * (y - target.y);
            double p2 = v1 * dx + v2 * dy;
            double j = p1 * p2;
            double h = p1 * (v1 * ddx + v2 * ddy) + p2 * p2;
            cur_s -= j / h;
            if (fabs(cur_s - prev_s) < 1e-5) break;
            prev_s = cur_s;
        }

        return std::min(cur_s, max_s);
    
    }
    


} // namespace Interplot
