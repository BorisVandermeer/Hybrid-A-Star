/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Test For functions in class "HybridAStar";
 * 
*********************************************************************/

#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>

#define _SEARCHER_HAS_DEBUG_MODE_
#include<HybridAStar.h>


#define IMG_FILENAME "./maps/map_basic.png"
#define MAP_UPSIZE_N 10

#define COLOR_WHITE Vec3b(255,255,255)
#define COLOR_BLACK Vec3b(0,0,0)
#define COLOR_GREEN Vec3b(0,255,0)
#define COLOR_RED   Vec3b(0,0,255)
#define COLOR_BLUE  Vec3b(255,0,0)

using Planning::Searcher::HybridAStar;
using Maps::GridMap;
using namespace cv;
using namespace std;

HybridAStar PlanHandler;
Mat MapImg = cv::imread(IMG_FILENAME,CV_8UC1);

void ShowGridMap(shared_ptr<Maps::GridMap> ptr , Mat & img2show){
    img2show = Mat(ptr->getSizeY(),ptr->getSizeX(),CV_8UC3,Vec3b(255,255,255));
    for(int x=0, maxx=ptr->getSizeX() ;x<maxx;x++)
        for(int y=0,maxy=ptr->getSizeY();y<maxy;y++){
            if(ptr->HasObstacle(x,y)) img2show.at<Vec3b>(y,x) = COLOR_BLACK;
            // else img2show.at<Vec3b>(y,x) = COLOR_WHITE;
        }
}

int main(){
    /* Load Data*/
    MapImg = imread(IMG_FILENAME);
    Planning::Searcher::HAS_DEBUG_FUNCTION();
    return 0;
}

Mat MapDataImg;
Mat MapImg2show;
shared_ptr<GridMap> MapData;

void Planning::Searcher::HAS_DEBUG_FUNCTION(){
    /*Load data*/
    MapDataImg = Mat::zeros(MapImg.rows * MAP_UPSIZE_N,MapImg.cols*MAP_UPSIZE_N, CV_8UC3);
    for(size_t i=0;i<MapDataImg.rows;i++) for(size_t j=0;j<MapDataImg.cols;j++){
        int ii = i/MAP_UPSIZE_N, jj = j/MAP_UPSIZE_N;
        if(MapImg.at<Vec3b>(ii,jj)==Vec3b(255,255,255))
             MapDataImg.at<Vec3b>(i,j) = {255,255,255};
    }

    /* cv Display windows config */
    namedWindow("Map Show",WINDOW_NORMAL);
    MapImg2show = MapDataImg.clone();
    // imshow("Map Show",MapImg2show);
    // while(1) if(waitKey(1)==27) break;
    /* Config Testing*/
    HybridAStar::VehicleShape data(4.7,2.0,1.3,2.7);
    // HybridAStar::VehicleShape data(.47,.20,.13,.27);
    PlanHandler.SetVehicleShape(data);

    HybridAStar::VehicleDynamicConfig config2;
    config2.Vehicle_Max_Steering = 10*M_PI/180;
    config2.Vehicle_Steering_disnum = 2;
    config2.Vehicle_Move_Stepsize = 0.1;
    config2.StateResolution = 1;
    config2.AngularGridSize = 72;
    config2.RSSteering = 10*M_PI/180;
    PlanHandler.SetVehicleDynamic(config2);

    HybridAStar::HybridAStarSearchConfig config1;
    config1.Segment_Len = 2.0;
    config1.OneShotDist = 5.0;
    config1.Penalty_Reversing = 3.0;
    config1.Penalty_Steering  = 1.05;
    config1.Penalty_SteeringChange = 1.5;
    PlanHandler.SetSearchConfig(config1);

    int size_x = MapDataImg.cols;
    int size_y = MapDataImg.rows;
    MapData = make_shared<GridMap>(size_x,size_y,1.0/MAP_UPSIZE_N);
    int a = 0;
    for(int x=0;x<size_x;x++) for(int y=0;y<size_y;y++){
        auto color = MapDataImg.at<Vec3b>(y,x);
        if(color == COLOR_BLACK){
            MapData->SetObstacleByIdx(x,y);
        }
    }

    PlanHandler.SetMap(MapData,config2.StateResolution);
    // ShowGridMap(PlanHandler.DilatedMap,MapImg2show);
    // imshow("Map Show",MapImg2show);
    // while(1) if(waitKey(1)==27) break;
    /* Config Test is Done*/


    /* Test Private Functions*/
    // First set search configs
    HybridAStar::Pos2d start(57.44673156738281,30-1.5998214721679688,-0.9655056865326423*M_PI);
    HybridAStar::Pos2d target(45.91535186767578,30-24.2706241607666,-0.7301509878243283*M_PI);
    auto start_grid  = PlanHandler.State2StateIndex(start);
    auto target_grid = PlanHandler.State2StateIndex(target);

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
    FromPtr->CostH = PlanHandler.ComputeH(FromPtr, GoalPtr);

    PlanHandler.From = FromPtr;
    PlanHandler.To = GoalPtr;
    PlanHandler.StateNodeMap[start_grid.x][start_grid.y][start_grid.phi] = FromPtr;
    PlanHandler.StateNodeMap[target_grid.x][target_grid.y][target_grid.phi] = GoalPtr;

    std::vector<HybridAStar::GridPos> NeighborIdxs;

    PlanHandler.GetNeighborNodes(FromPtr,NeighborIdxs);
    
    MapImg2show = Mat(MapData->getSizeY(),MapData->getSizeX(),CV_8UC3,Vec3b(255,255,255));
    ShowGridMap(MapData,MapImg2show);

    for(auto &t :NeighborIdxs){
        auto &points = PlanHandler.StateNodeMap[t.x][t.y][t.phi]->intermediate_states_;
        int maxi=points.size();
        for(int i=0;i<maxi;i++){
            auto point = points[i];
            auto idx = MapData->ToIdx(point.x,point.y);
            MapImg2show.at<Vec3b>(idx[1],idx[0]) = COLOR_GREEN;
        }
    }

    // imshow("Map Show",MapImg2show);
    // while(1) if(waitKey(1)==27) break;
    

    PlanHandler.Reset();
    PlanHandler.Search(start, target);
    GoalPtr = PlanHandler.StateNodeMap[target_grid.x][target_grid.y][target_grid.phi];

    while(GoalPtr->parent_ptr!=nullptr){
        auto &points = GoalPtr->intermediate_states_;
        int maxi=points.size();
        for(int i=0;i<maxi;i++){
            auto point = points[i];
            auto idx = MapData->ToIdx(point.x,point.y);
            MapImg2show.at<Vec3b>(idx[1],idx[0]) = COLOR_RED;
        }
        GoalPtr = GoalPtr->parent_ptr;
    }

    // imshow("Map Show",MapImg2show);
    // while(waitKey(0)!=27){};


    return;
}