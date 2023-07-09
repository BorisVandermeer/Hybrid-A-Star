/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A implement of Planning node of Search algorithm
 * 
 * Reference : 
 *      https://github.com/zm0612/Hybrid_A_Star
 * 
*********************************************************************/
#ifndef _PLANNING_STATE_NODE_H_
#define _PLANNING_STATE_NODE_H_

#include<memory>
#include<vector>
#include<Points/PointsType.h>

namespace Planning
{
    class StateNode
    {
    public:
        enum NODE_STATUS {
            NOT_VISITED = 0, IN_OPENSET = 1, IN_CLOSESET = 2
        };

        enum DIRECTION {
            FORWARD = 0, BACKWARD = 1, NO = 3
        };

        typedef Points::Pos2D     Pos2d;
        typedef Points::GridPos2D GridPos;
        typedef Points::Pos2Ds    Pos2ds;

        StateNode() = delete;
        explicit StateNode(const GridPos &grid_index) {
            status = NOT_VISITED;
            GridIdx = grid_index;
            parent_ptr = nullptr;
        }

        NODE_STATUS status;
        DIRECTION   dir;

        GridPos GridIdx;
        int     SteerIdx;
        
        Pos2d   State;
        double  Steering;

        double CostG, CostH;
        
        
        std::shared_ptr<StateNode> parent_ptr;
        typedef std::shared_ptr<StateNode> Ptr;

        Pos2ds intermediate_states_;
    };
    
} // namespace Planning

#endif