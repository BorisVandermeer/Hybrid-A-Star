/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A implement of Normal Vehicle Models
 * 
*********************************************************************/

#include<vector>
#include<Points/PointsType.h>

namespace Models{

    class VehicleDynamicState{
        double x,y,heading;
        double v_par,v_vel;
        double a_par,a_vel;
        double phi;
    };

    class VehicleShape{
    public:
        VehicleShape(){};
        VehicleShape(double _len, double _wid, double _reardist,double _wheelbase)
                :len(_len),wid(_wid),reardist(_reardist),wheelbase(_wheelbase){}
        ~VehicleShape(){};
        
        /*
        * @param len vehicle length (a to c)
        * @param wid vehicle width (a to d)
        * @param reardist Length from rear axle to rear (a to b)
        * @param wheelbase lenth between front axle to rear axle 
        * 
        *       b          e
        *  a  ----------------- c
        *    |  |          |  |    Front
        *    |  |          |  |
        *  d  -----------------
        */
        double len,wid,reardist,wheelbase;
    };

    class VehicleMoveModel : public VehicleShape{
    public :
        VehicleMoveModel(){};
        VehicleMoveModel(double _len, double _wid, double _reardist,double _wheelbase){
            len = _len,wid = _wid,reardist = _reardist,wheelbase = _wheelbase;
        }
        VehicleMoveModel(VehicleShape shape){
            len = shape.len,wid = shape.wid,reardist = shape.reardist,wheelbase = shape.wheelbase;
        }
        
        typedef Points::Pos2D Pos2d;

        double Steer2Radius(double const &angle) const;
        Pos2d MoveByRadius(Pos2d const &from, double const &radius, double const &dist)  const;
        Pos2d MoveBySteerting(Pos2d const &from, double const &angle, double const &dist) const;
        
        
    };

} // namespace Models
