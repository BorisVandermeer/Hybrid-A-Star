/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A implement of Hybrid A Star algorithm.
 *      Use " #define _SEARCHER_HAS_DEBUG_MODE_ " to Enable a friedn DEBUG function.
 * 
 * Reference : 
 *      https://github.com/zm0612/Hybrid_A_Star
 * 
*********************************************************************/
#ifndef _PLANNING_HYBRID_A_STAR_H_
#define _PLANNING_HYBRID_A_STAR_H_

#include<memory>
#include<map>

#include<Curves/ReedsShepp.h>

#include<VehicleCircleShape.hpp>
#include<GridMap.h>
#include<StateNode.h>
#include<Points/RefPointsGenerator.h>

namespace Planning
{

namespace Searcher
{
    namespace HybridAStarGlobal{
        extern int Radius; // Pix Rad
        void DilateKernal(Maps::GridMap::MapDataType * pData, int x, int y);
    }

    class HybridAStar {
    public:
        struct HybridAStarSearchConfig{
            double Segment_Len;
            double OneShotDist;
            double Penalty_SteeringChange;
            double Penalty_Steering;
            double Penalty_Reversing ;
        };

        struct VehicleDynamicConfig{
            double Vehicle_Move_Stepsize;
            double Vehicle_Max_Steering;
            double StateResolution;
            int AngularGridSize;
            int Vehicle_Steering_disnum;
            double RSSteering;
        };

        typedef Models::VehicleShape VehicleShape;
        typedef Maps::GridMap SearchMap;

        // Should be Called Before SetMap, and Dynamic.
        void SetVehicleShape(VehicleShape const &data);
        void SetVehicleDynamic(VehicleDynamicConfig const &data);
        void SetSearchConfig(HybridAStarSearchConfig &config);
        void SetMap(std::shared_ptr<SearchMap>& map_ptr,double state_grid_resolution);
        
        
        typedef StateNode::GridPos GridPos;
        typedef StateNode::Pos2d   Pos2d;
        typedef StateNode::Pos2ds  Pos2ds;

        bool Search( const Pos2d   & start, const Pos2d   & target);
        // bool Search( const GridPos & start, const GridPos & target);

        Pos2ds GetResult();
        void Reset();

    private:
        GridPos State2StateIndex(Pos2d state);
        virtual double ComputeH(StateNode::Ptr const &pStart);
        virtual double ComputeH(StateNode::Ptr const &pStart, StateNode::Ptr const &pTarget);
        virtual double ComputeG(StateNode::Ptr const &pStart, StateNode::Ptr const &pNeighbor);
        virtual bool HasCollision(Pos2d  const & State);
        virtual bool Expand(StateNode::Ptr const &pCurrent, StateNode::Ptr const &pTarget);
        virtual void GetNeighborNodes(const StateNode::Ptr &CurrentPtr, std::vector<GridPos> &NeighborIdxs);
       
        StateNode::Ptr From = nullptr;
        StateNode::Ptr To = nullptr;
        std::vector<std::vector<std::vector<StateNode::Ptr>>> StateNodeMap;
        

    /* Tools*/
        PointsTools::PointsGenHandller PointsHandler;

    /* Vehicle Config*/
    private:
        typedef Models::VehicleShapeCircles<2> VehicleShape_;
        typedef Models::VehicleMoveModel VehicleDynamicModel;
        std::shared_ptr<SearchMap> DilatedMap = nullptr;
        VehicleShape_ VehicleData = VehicleShape(4.8,1.8,1.0,2.8);
        VehicleDynamicModel ModelData = VehicleShape(4.8,1.8,1.0,2.8);
        
        double Vehicle_Move_Stepsize;
        double Vehicle_Max_Steering;
        int    Vehicle_Steering_disnum;
    
    /* Search Penalty Config*/
    private :
        double Segment_Len;
        double OneShotDist;
        double Penalty_SteeringChange = 1.0;
        double Penalty_Steering;
        double Penalty_Reversing;
        

    /* Search Curves Config*/
    private:
        typedef Curves::RSCurveStateSpace PathSpace;
        std::shared_ptr<PathSpace> RSCurveSpace;
    
    /*Map Config*/
    private:
        std::shared_ptr<SearchMap> MapData = nullptr;
        size_t MAP_GRID_SIZE_X_;
        size_t MAP_GRID_SIZE_Y_;
        double MAP_GRID_RESOLUTION_;     //meters per pix
    
    /* Search Map Config */
    private:
        double STATE_GRID_RESOLUTION_;   //meters per pix
        double ANGULAR_RESOLUTION_;
        size_t STATE_GRID_SIZE_X_;
        size_t STATE_GRID_SIZE_Y_;
        size_t STATE_GRID_SIZE_PHI_;  // 0 for -pi max for pi-res

#ifdef _SEARCHER_HAS_DEBUG_MODE_
    friend void HAS_DEBUG_FUNCTION();

#endif

    };
#ifdef _SEARCHER_HAS_DEBUG_MODE_
    void HAS_DEBUG_FUNCTION();
#endif

} // namespace Searcher

} // namespace Planning


#endif