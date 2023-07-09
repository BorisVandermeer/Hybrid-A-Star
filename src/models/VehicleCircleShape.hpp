/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Descripe Vehicle with servel shapes
 * 
*********************************************************************/

#include<VehicleModel.h>

namespace Models{
    
    template<int N = 2>
    class VehicleShapeCircles : private VehicleShape{
    public:
        VehicleShapeCircles() = delete;
        VehicleShapeCircles(VehicleShape _shape);
        double Radius;
        std::vector<double> CircleCenters; // Distance to rear axle center;
    };

    template<int N>
    VehicleShapeCircles<N>::VehicleShapeCircles(VehicleShape _shape){
        len = _shape.len, wid=_shape.wid, reardist = _shape.reardist, wheelbase = _shape.wheelbase;
        const int &nums = N;
        CircleCenters.resize(nums);
        double stepsize = len/nums;
        Radius = std::sqrt(stepsize*stepsize+wid*wid)/2;
        CircleCenters[0] = stepsize/2-reardist;
        for(int i=1;i<nums;i++){
            CircleCenters[i] = CircleCenters[i-1]+stepsize;
        }
    }

    typedef VehicleShapeCircles<2> VehicleShape2Circles;
    typedef VehicleShapeCircles<3> VehicleShape3Circles;

} // namespace Models