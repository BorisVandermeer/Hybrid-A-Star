/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A implement of Hybrid A Star algorithm.
 * 
 * Reference : 
 *      https://github.com/zm0612/Hybrid_A_Star
 * 
*********************************************************************/

#include<queue>
#include<cmath>
// Utils
#include<Points/RefPointsGenerator.h>
#include<HybridAStar.h>

// using namespace std;


namespace Planning
{
namespace Searcher
{
    namespace Utils{

        inline static double Distance(double x1,double x2,double y1,double y2){
            return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
        }

        inline static double Distance(StateNode::Pos2d& start, StateNode::Pos2d& target) {
            return Distance(start.x,target.x,start.y,target.y);
        }
        
        inline static double Distance(StateNode& start, StateNode& target){
            return Distance(start.State,target.State);
        }

        inline static double Distance(StateNode::Ptr pstart, StateNode::Ptr ptarget){
            return Distance(*pstart,*ptarget);
        }

        // TOo [-pi, pi]
        inline static double Mod2Pi(const double &x){
            double v = fmod(x, 2 * M_PI);
            if (v < -M_PI) {
            v += 2.0 * M_PI;
            } else if (v >= M_PI) {
                v -= 2.0 * M_PI;
            }
            return v;
        }
    }

    void HybridAStar::Reset(){
        StateNodeMap.clear();
        StateNodeMap.resize(STATE_GRID_SIZE_X_);
        for(auto it = StateNodeMap.begin();it!=StateNodeMap.end();it++){
            it->resize(STATE_GRID_SIZE_Y_);
            for(auto it2 = it->begin();it2!=it->end();it2++){
                it2->resize(STATE_GRID_SIZE_PHI_);
                for(auto it3 = it2->begin();it3!=it2->end();it3++){
                    *it3 = nullptr;
                }
            }
        }
    }

    void HybridAStar::SetMap(std::shared_ptr<SearchMap>& map_ptr,double state_grid_resolution)
    {
        MapData = map_ptr;
        MAP_GRID_RESOLUTION_ = MapData->getReslotion();
        STATE_GRID_RESOLUTION_ = state_grid_resolution;
        MAP_GRID_SIZE_X_ = MapData->getSizeX();
        MAP_GRID_SIZE_Y_ = MapData->getSizeY();
        STATE_GRID_SIZE_X_ = std::floor(MapData->getMaxX()/STATE_GRID_RESOLUTION_);
        STATE_GRID_SIZE_Y_ = std::floor(MapData->getMaxY()/STATE_GRID_RESOLUTION_);
        Reset();
        HybridAStarGlobal::Radius = VehicleData.Radius/MAP_GRID_RESOLUTION_+1;
        MapData->Dilate(DilatedMap,HybridAStarGlobal::DilateKernal);
    }

    void HybridAStar::SetSearchConfig(HybridAStarSearchConfig &config){
        OneShotDist = config.OneShotDist;
        Segment_Len = config.Segment_Len;
        Penalty_Steering = config.Penalty_Steering;
        Penalty_Reversing = config.Penalty_Reversing;
    }

    HybridAStar::GridPos HybridAStar::State2StateIndex(Pos2d state){
        GridPos ans;
        double phi = Utils::Mod2Pi(state.phi);
        ans.x = state.x/STATE_GRID_RESOLUTION_;
        ans.y = state.y/STATE_GRID_RESOLUTION_;
        ans.phi = (phi+M_PI)/ANGULAR_RESOLUTION_;

        ans.x   = std::max(0,ans.x  ); ans.x   = std::min(static_cast<int>(STATE_GRID_SIZE_X_-1  ),ans.x  );
        ans.y   = std::max(0,ans.y  ); ans.y   = std::min(static_cast<int>(STATE_GRID_SIZE_Y_-1  ),ans.y  );
        // ans.phi = std::max(0,ans.phi); ans.phi = std::min(static_cast<int>(STATE_GRID_SIZE_PHI_),ans.phi);
        
        return ans;
    }

    double HybridAStar::ComputeH(StateNode::Ptr const &pStart){
        return ComputeH(pStart,To);
    }

    double HybridAStar::ComputeH(StateNode::Ptr const &pStart, StateNode::Ptr const &pTarget){
        double ans = 0;
        // Distance to Target
        double distance = Utils::Distance(pStart,pTarget);
        auto s = RSCurveSpace->RSCurveCalc(pStart->State,pTarget->State);
        for(int i=0;i<5;i++){
            double cost = 0;
            if(s.type_[i]==Curves::RSCurveSegmentType::RS_LEFT||s.type_[i]==Curves::RSCurveSegmentType::RS_LEFT){
                cost+=fabs(s.length[i])*Penalty_Steering;
            } else if(s.type_[i]==Curves::RSCurveSegmentType::RS_STRAIGHT) {
                cost+=fabs(s.length[i]);
            }
            if(s.length[i]<0) 
                cost*=Penalty_Reversing;
            ans+=cost;
        }
        return ans;
        // pStart->cost_h = ans;
    }
    
    double HybridAStar::ComputeG(StateNode::Ptr const &pStart, StateNode::Ptr const &pTarget){
        double ans = 0;
        if (pTarget->dir == StateNode::FORWARD) {
        if (pTarget->Steering != pStart->Steering) {
            if (pTarget->Steering == 0) {
                ans = Segment_Len * Penalty_SteeringChange;
            } else {
                ans = Segment_Len * Penalty_SteeringChange * Penalty_Steering;
            }
        } else {
            if (pTarget->Steering == 0) {
                ans = Segment_Len;
            } else {
                ans = Segment_Len * Penalty_Steering;
            }
        }
        } else {
            if (pTarget->Steering != pStart->Steering) {
                if (pTarget->Steering == 0) {
                    ans = Segment_Len * Penalty_SteeringChange * Penalty_Reversing;
                } else {
                    ans = Segment_Len * Penalty_SteeringChange * Penalty_Steering * Penalty_Reversing;
                }
            } else {
                if (pTarget->Steering == 0) {
                    ans = Segment_Len * Penalty_Reversing;
                } else {
                    ans = Segment_Len * Penalty_Steering * Penalty_Reversing;
                }
            }
        }
        return ans;
    }

namespace HybridAStarGlobal{
    int Radius = 10; // Pix Rad
    inline static void Set(Maps::GridMap::MapDataType *pData, int &x, int &y){
        int size_x = pData->size();
        int size_y = pData->back().size();
        if(x>0&&x<size_x &&y>0&&y<size_y)
            (*pData)[x][y] = GRIDMAP_IS_OBSTACLE;
    }

    void DilateKernal(Maps::GridMap::MapDataType * pData, int x, int y){
        int size_x = pData->size();
        int size_y = pData->front().size();
        int x_min = x-Radius>0 ? x-Radius : 0;
        int x_max = x+Radius<size_x-1 ? x+Radius : size_x-1;
        int y_min = y-Radius>0 ? y-Radius : 0;
        int y_max = y+Radius<size_y-1 ? y+Radius : size_y-1;
        int r2 = Radius*Radius;

        for(int xx = x_min;xx<=x_max;xx++) for(int yy = y_min;yy<=y_max;yy++) {
            int tmpr2 = (xx-x)*(xx-x)+(yy-y)*(yy-y);
            if(tmpr2<=r2) (*pData)[xx][yy] = GRIDMAP_IS_OBSTACLE;
        }
    }
}

    void HybridAStar::SetVehicleShape(VehicleShape const &data){
        VehicleData = data;
        ModelData = data;
        HybridAStarGlobal::Radius = VehicleData.Radius/MAP_GRID_RESOLUTION_+1;
        if(MapData!=nullptr)
            MapData->Dilate(DilatedMap,HybridAStarGlobal::DilateKernal);
        
    };

    void HybridAStar::SetVehicleDynamic(VehicleDynamicConfig const &data){
        Vehicle_Move_Stepsize = data.Vehicle_Move_Stepsize;
        Vehicle_Max_Steering = data.Vehicle_Max_Steering;
        Vehicle_Steering_disnum = data.Vehicle_Steering_disnum;
        STATE_GRID_RESOLUTION_ = data.StateResolution;
        STATE_GRID_SIZE_PHI_ = data.AngularGridSize;
        ANGULAR_RESOLUTION_  = 2*M_PI/STATE_GRID_SIZE_PHI_;
        RSCurveSpace = std::make_shared<PathSpace>(ModelData.Steer2Radius(data.RSSteering));
    }

    bool HybridAStar::HasCollision(Pos2d const & State){
        for(int i=0,maxi = VehicleData.CircleCenters.size();i<maxi;i++){
            double x = State.x+VehicleData.CircleCenters[i]*std::cos(State.phi);
            double y = State.y+VehicleData.CircleCenters[i]*std::sin(State.phi);
            if(x<VehicleData.Radius||y<VehicleData.Radius) return true;
            if(x>MAP_GRID_SIZE_X_*MAP_GRID_RESOLUTION_-VehicleData.Radius 
               || y>MAP_GRID_SIZE_Y_*MAP_GRID_RESOLUTION_-VehicleData.Radius) return true;
            size_t idx_x = x/MAP_GRID_RESOLUTION_; size_t idx_y = y/MAP_GRID_RESOLUTION_;
            if(DilatedMap->HasObstacle(idx_x,idx_y)) return true;
        }
        return false;
    }

    bool HybridAStar::Expand(StateNode::Ptr const &pCurrent, StateNode::Ptr const &pTarget){
        auto s = RSCurveSpace->RSCurveCalc(pCurrent->State,pTarget->State);
        Points::Pos2Ds poses;
        PointsHandler.GetPoses(poses,s,Vehicle_Move_Stepsize,pCurrent->State);
        // Points::Pos2Ds poses;
        for(int i=0,maxi = poses.size();i<maxi;i++){
            if(HasCollision(Pos2d(poses.x[i],poses.y[i],poses.phi[i]))) return false;
        }
        pTarget->intermediate_states_ = poses;
        pTarget->parent_ptr = pCurrent;

        return true;
    }

    void HybridAStar::GetNeighborNodes(const StateNode::Ptr &CurPtr, std::vector<GridPos> &Neighbors){
        Neighbors.clear();
        for(int i = -Vehicle_Steering_disnum;i<=Vehicle_Steering_disnum;i++){
            // double x = CurPtr->State.x;
            // double y = CurPtr->State.y;
            // double phi = CurPtr->State.phi;
            
            StateNode::Pos2ds intermediate_states;

            const double steer = i * Vehicle_Max_Steering/Vehicle_Steering_disnum;
            bool HasObstacle = false;
            int maxj = Segment_Len/Vehicle_Move_Stepsize;
            // Forward
            Pos2d curpos(CurPtr->State); 
            for (int j = 1; j <= maxj; j++) {
                curpos = ModelData.MoveBySteerting(curpos,steer,Vehicle_Move_Stepsize);
                intermediate_states.push_back(curpos);
                if (HasCollision(curpos)){
                    HasObstacle = true;
                    break;
                }
            }

            if(!HasObstacle){
                GridPos curidx = State2StateIndex(curpos);
                    StateNode::Ptr pcur = std::make_shared<StateNode>(curidx);
                    pcur->dir = StateNode::FORWARD;
                    pcur->parent_ptr = CurPtr;
                    pcur->State = curpos;
                    pcur->intermediate_states_ = intermediate_states;
                    pcur->CostH = CurPtr->CostG + ComputeH(pcur);
                    pcur->CostG = CurPtr->CostG + ComputeG(CurPtr,pcur);
                if(StateNodeMap[curidx.x][curidx.y][curidx.phi] == nullptr){
                    StateNodeMap[curidx.x][curidx.y][curidx.phi] = pcur;
                    Neighbors.push_back(curidx);
                } else if((StateNodeMap[curidx.x][curidx.y][curidx.phi]->CostG)<(pcur->CostG)){
                    StateNodeMap[curidx.x][curidx.y][curidx.phi] = pcur;
                    Neighbors.push_back(curidx);
                } else{}
            }

            // Backward
            curpos = CurPtr->State; 
            HasObstacle = false;
            for (int j = 1; j <= maxj; j++) {
                curpos = ModelData.MoveBySteerting(curpos,steer,-Vehicle_Move_Stepsize);
                intermediate_states.push_back(curpos);
                if (HasCollision(curpos)){
                    HasObstacle = true;
                    break;
                }
            }

            if(!HasObstacle){
                GridPos curidx = State2StateIndex(curpos);
                    StateNode::Ptr pcur = std::make_shared<StateNode>(curidx);
                    pcur->dir = StateNode::BACKWARD;
                    pcur->parent_ptr = CurPtr;
                    pcur->State = curpos;
                    pcur->intermediate_states_ = intermediate_states;
                    pcur->CostH = CurPtr->CostG + ComputeH(pcur);
                    pcur->CostG = CurPtr->CostG + ComputeG(CurPtr,pcur);
                if(StateNodeMap[curidx.x][curidx.y][curidx.phi] == nullptr){
                    StateNodeMap[curidx.x][curidx.y][curidx.phi] = pcur;
                    Neighbors.push_back(curidx);
                } else if(StateNodeMap[curidx.x][curidx.y][curidx.phi]->status!=StateNode::IN_CLOSESET
                            &&(StateNodeMap[curidx.x][curidx.y][curidx.phi]->CostG)<(pcur->CostG)){
                    StateNodeMap[curidx.x][curidx.y][curidx.phi] = pcur;
                    Neighbors.push_back(curidx);
                } else{}
            }
        }
    }


    bool HybridAStar::Search( const Pos2d & start, const Pos2d & target){
        GridPos start_grid  = State2StateIndex(start);
        GridPos target_grid = State2StateIndex(target);

        StateNode::Ptr GoalPtr(new StateNode(target_grid));
        GoalPtr->State = target;
        GoalPtr->dir = StateNode::NO;
        GoalPtr->Steering = 0;

        StateNode::Ptr FromPtr(new StateNode(start_grid));
        FromPtr->State = start;
        FromPtr->Steering = 0;
        FromPtr->dir = StateNode::NO;
        FromPtr->status = StateNode::IN_OPENSET;
        FromPtr->intermediate_states_.clear();
        FromPtr->CostG = 0.0;
        FromPtr->CostH = ComputeH(FromPtr, GoalPtr);

        From = FromPtr;
        To = GoalPtr;

        StateNodeMap[start_grid.x][start_grid.y][start_grid.phi] = FromPtr;
        StateNodeMap[target_grid.x][target_grid.y][target_grid.phi] = GoalPtr;

        struct CompareStruct{
            int idx1,idx2,idx3;
            double val;
            CompareStruct(int x, int y,int phi,double _val = 0):idx1(x),idx2(y),idx3(phi),val(_val){}
            CompareStruct(GridPos p,double _val = 0):idx1(p.x),idx2(p.y),idx3(p.phi),val(_val){}
        };
        
        class greater{
        public:
            bool operator()(const CompareStruct &n1, const CompareStruct & n2){
                if(n1.val>n2.val) return true;
                return false;
            }
        };

        std::priority_queue<CompareStruct, std::vector<CompareStruct>, greater> openset_;
        openset_.push(CompareStruct(start_grid,0));

        std::vector<GridPos> NeighborIdxs;
        StateNode::Ptr CurrentPtr;
        StateNode::Ptr Neighbors;
        int count = 0;
        while (!openset_.empty()) {
            auto &tmp = openset_.top();
            CurrentPtr = StateNodeMap[tmp.idx1][tmp.idx2][tmp.idx3]; 
            openset_.pop();

            if(Utils::Distance(CurrentPtr,GoalPtr)<OneShotDist){
                if(Expand(CurrentPtr,GoalPtr)){
                    return true;
                }
            }

            if(CurrentPtr->status == StateNode::IN_CLOSESET) continue;
            CurrentPtr->status = StateNode::IN_CLOSESET;
            GetNeighborNodes(CurrentPtr, NeighborIdxs);
            for(auto & ptmp : NeighborIdxs){
                CompareStruct tmpdata(ptmp);
                auto & ptr = StateNodeMap[tmpdata.idx1][tmpdata.idx2][tmpdata.idx3];
                tmpdata.val = ptr->CostG + ptr->CostH;
                ptr ->status = StateNode::IN_OPENSET;
                openset_.push(tmpdata);
            }

            count++;
            if (count > 50000) {
                return false;
            }
        }

        return false;

    }

    HybridAStar::Pos2ds HybridAStar::GetResult(){
        auto GoalPtr = To;
        Pos2ds ans;
        while(GoalPtr->parent_ptr!=nullptr){
            auto &points = GoalPtr->intermediate_states_;
            int maxi=points.size();
            for(int i=0;i<maxi;i++){
                ans.push_back(points[i]);
            }
            GoalPtr = GoalPtr->parent_ptr;
        }
        return ans;
    }

void __attribute__((weak)) HAS_DEBUG_FUNCTION(){};


} // namespace Searcher
} // namespace Planning
