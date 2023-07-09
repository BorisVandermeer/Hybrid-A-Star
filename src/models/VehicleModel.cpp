/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A implement of Normal Vehicle Models
 * 
*********************************************************************/

#include<cmath>

#include<VehicleModel.h>


namespace Models{

    static double Mod2Pi(const double &x){
        double v = fmod(x, 2 * M_PI);
        if (v < -M_PI) {
            v += 2.0 * M_PI;
        } else if (v > M_PI) {
            v -= 2.0 * M_PI;
        }
        return v;
    }

    VehicleMoveModel::Pos2d VehicleMoveModel::MoveByRadius(Pos2d const &from, double const &radius, double const &dist) const{
        double v = dist/radius;
        Pos2d ans;
        if(v>0){
            ans.x = from.x + (std::sin(from.phi + v) - std::sin(from.phi))*radius;
            ans.y = from.y - (std::cos(from.phi + v) - std::cos(from.phi))*radius;
            ans.phi = Mod2Pi(from.phi + v);
        } else {
            ans.x = from.x - (std::sin(from.phi - v) - std::sin(from.phi))*radius;
            ans.y = from.y + (std::cos(from.phi - v) - std::cos(from.phi))*radius;
            ans.phi = Mod2Pi(from.phi + v);
        }
        
        return ans;
    }
    
    double VehicleMoveModel::Steer2Radius(double const &angle) const{
        if(fabs(angle)<0.000001) return __DBL_MAX__;
        else return  wheelbase/std::tan(angle);
    }

    VehicleMoveModel::Pos2d VehicleMoveModel::MoveBySteerting(Pos2d const &from, double const &angle, double const &dist) const{
        if(fabs(angle)<0.000001) 
            return Pos2d(from.x + dist*std::cos(from.phi),from.y + dist*std::sin(from.phi),from.phi+angle);
        double radius = wheelbase/std::tan(angle);
        return MoveByRadius(from,radius,dist);
    }

} // namespace Models