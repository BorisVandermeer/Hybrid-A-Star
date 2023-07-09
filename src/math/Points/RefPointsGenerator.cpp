/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      Generate States Form a referance Path
 * 
*********************************************************************/

#include<assert.h>
#include<Points/RefPointsGenerator.h>

using namespace std;
namespace PointsTools{

    Interplot::SplineCurve PointsGenHandller::toSplineCurve(RefPoints & points) const{
        Interplot::SplineCurve ans;
        ans.setpoints(points);
        return ans;
    }
        
    bool PointsGenHandller::GetPoses(RefPoses & poses, Curves::RSCurve const RSC, double stepsize,StartPos2d From){
        // double phi = 0;
        size_t num = RSC.TotalLength()/stepsize+1;
        double seglen = RSC.TotalLength()/RSC.radius;
        
        for(double i=0;i<=num;i++){
            double v,seg = static_cast<double>(i)*seglen/num;
            double tmpx = 0,tmpy = 0,tmpphi= From.phi;
            for(int j=0;j<5&&seg>0;j++){
                if(RSC.length[j]<0.0){
                    v = std::max(-seg, RSC.length[j]);
                    seg += v;
                } else {
                    v = std::min(seg, RSC.length[j]);
                    seg -= v;
                }

                switch(RSC.type_[j]){
                    case Curves::RSCurveSegmentType::RS_LEFT:
                        tmpx = tmpx + std::sin(tmpphi + v) - std::sin(tmpphi);
                        tmpy = tmpy - std::cos(tmpphi + v) + std::cos(tmpphi);
                        tmpphi = tmpphi + v;
                        break;
                    case Curves::RSCurveSegmentType::RS_RIGHT:
                        tmpx = tmpx - std::sin(tmpphi - v) + std::sin(tmpphi);
                        tmpy = tmpy + std::cos(tmpphi - v) - std::cos(tmpphi);
                        tmpphi = tmpphi - v;
                        break;
                    case Curves::RSCurveSegmentType::RS_STRAIGHT:
                        tmpx = v * std::cos(tmpphi) + tmpx;
                        tmpy = v * std::sin(tmpphi) + tmpy;
                        break;
                    case Curves::RSCurveSegmentType::RS_NOP:
                        break;
                }
            }
            poses.x.push_back(tmpx*RSC.radius+From.x);
            poses.y.push_back(tmpy*RSC.radius+From.y);
            poses.phi.push_back(tmpphi);
        }

        return true;
        
    }
}// namespace Utils