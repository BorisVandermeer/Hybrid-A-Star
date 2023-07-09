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

static void __attribute__((unused)) dilate(Vec3b Color,int n,Mat & img)
{
    Mat img_backup = img.clone();
    int & rows = img_backup.rows;
    int & cols = img_backup.cols;
    for(int row = n;row<rows-n;row++){
        for(int col = n;col<cols-n;col++){
            bool flag = false;
            for(int i = -n;i<=n;i++){
                for(int j=-n;j<=n;j++){
                    if(img_backup.at<Vec3b>(row+i,col+j) == Color){
                        img.at<Vec3b>(row,col) = Color;
                        flag = true;
                        break;
                    }
                }
                if(flag) break;
            }
        }
    }
}

void ShowGridMap(shared_ptr<Maps::GridMap> ptr , Mat & img2show){
    img2show = Mat(ptr->getSizeY(),ptr->getSizeX(),CV_8UC3,Vec3b(255,255,255));
    for(int x=0, maxx=ptr->getSizeX() ;x<maxx;x++)
        for(int y=0,maxy=ptr->getSizeY();y<maxy;y++){
            if(ptr->HasObstacle(x,y)) img2show.at<Vec3b>(y,x) = COLOR_BLACK;
            else img2show.at<Vec3b>(y,x) = COLOR_WHITE;
        }
}

int main(){
    /* Load Data*/
    MapImg = imread(IMG_FILENAME);
    Planning::Searcher::HAS_DEBUG_FUNCTION();
    return 0;
}

Mat MapDataImg;
Mat Img2show;
shared_ptr<GridMap> MapData;
HybridAStar::Pos2d start;
HybridAStar::Pos2d target;

int cnt = 0;
void EventMouseClick(int event, int xc, int yc, int flags, void* ustc){
    if(event == EVENT_LBUTTONDOWN){
        double res = MapData->getReslotion();
        double posx = xc*res;
        double posy = yc*res;
        ShowGridMap(MapData,Img2show);
        switch(cnt){
            case 0:
                start.x = posx;
                start.y = posy;
                Img2show.at<Vec3b>(start.y/res,start.x/res) = COLOR_GREEN;
                dilate(COLOR_GREEN,2,Img2show);
                cnt++;
                break;
            case 1:
                start.phi = atan2(posy-start.y,posx-start.x);
                Img2show.at<Vec3b>(start.y/res,start.x/res) = COLOR_GREEN;
                Img2show.at<Vec3b>(start.y/res+2/res*sin(start.phi),start.x/res+2/res*cos(start.phi)) = COLOR_GREEN;
                dilate(COLOR_GREEN,2,Img2show);
                cnt++;
                break;
            case 2:
                target.x = posx;
                target.y = posy;
                Img2show.at<Vec3b>(start.y/res,start.x/res) = COLOR_GREEN;
                Img2show.at<Vec3b>(start.y/res+2/res*sin(start.phi),start.x/res+2/res*cos(start.phi)) = COLOR_GREEN;
                Img2show.at<Vec3b>(target.y/res,target.x/res) = COLOR_GREEN;
                dilate(COLOR_GREEN,2,Img2show);
                cnt++;
                break;
            case 3:
                target.phi = atan2(posy-target.y,posx-target.x);
                Img2show.at<Vec3b>(yc,xc) = COLOR_GREEN;
                Img2show.at<Vec3b>(start.y/res,start.x/res) = COLOR_GREEN;
                Img2show.at<Vec3b>(start.y/res+2/res*sin(start.phi),start.x/res+2/res*cos(start.phi)) = COLOR_GREEN;
                Img2show.at<Vec3b>(target.y/res,target.x/res) = COLOR_GREEN;
                dilate(COLOR_GREEN,2,Img2show);
                cnt++;
                break;
            default:
                break;
        }
        imshow("Map Show",Img2show);
    } else if(event == EVENT_RBUTTONDOWN){
        ShowGridMap(MapData,Img2show);
        cnt = 0;
        imshow("Map Show",Img2show);
    }

}

void Planning::Searcher::HAS_DEBUG_FUNCTION(){
    /*Load data*/
    MapDataImg = Mat::zeros(MapImg.rows * MAP_UPSIZE_N,MapImg.cols*MAP_UPSIZE_N, CV_8UC3);
    for(int i=0;i<MapDataImg.rows;i++) for(int j=0;j<MapDataImg.cols;j++){
        int ii = i/MAP_UPSIZE_N, jj = j/MAP_UPSIZE_N;
        if(MapImg.at<Vec3b>(ii,jj)==Vec3b(255,255,255))
             MapDataImg.at<Vec3b>(i,j) = {255,255,255};
    }

    /* cv Display windows config */
    namedWindow("Map Show",WINDOW_NORMAL);
    Img2show = MapDataImg.clone();
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
    for(int x=0;x<size_x;x++) for(int y=0;y<size_y;y++){
        auto color = MapDataImg.at<Vec3b>(y,x);
        if(color == COLOR_BLACK){
            MapData->SetObstacleByIdx(x,y);
        }
    }
    PlanHandler.SetMap(MapData,config2.StateResolution);

    // MapData = PlanHandler.DilatedMap;
    /* Test Private Functions*/
    ShowGridMap(MapData,Img2show);
    imshow("Map Show",Img2show);
    setMouseCallback("Map Show",EventMouseClick);
    while(waitKey(2)!=27){
        if(cnt == 4){
            PlanHandler.Reset();
            if(!(PlanHandler.Search(start, target))){
                cout<<"Plan Failed"<<endl;
            } else {
                // HybridAStar::GridPos start_grid  = PlanHandler.State2StateIndex(start);
                HybridAStar::GridPos target_grid = PlanHandler.State2StateIndex(target);
                auto GoalPtr = PlanHandler.StateNodeMap[target_grid.x][target_grid.y][target_grid.phi];

                // while(GoalPtr->parent_ptr!=nullptr){
                //     auto &points = GoalPtr->intermediate_states_;
                //     int maxi=points.size();
                //     for(int i=0;i<maxi;i++){
                //         auto point = points[i];
                //         auto idx = MapData->ToIdx(point.x,point.y);
                //         Img2show.at<Vec3b>(idx[1],idx[0]) = COLOR_RED;
                //     }
                //     GoalPtr = GoalPtr->parent_ptr;
                // }

                auto points = PlanHandler.GetResult();
                for(int i=0;i<points.size();i++){
                    auto point = points[i];
                    auto idx = MapData->ToIdx(point.x,point.y);
                    Img2show.at<Vec3b>(idx[1],idx[0]) = COLOR_RED;
                }
                
                imshow("Map Show",Img2show);
            }
            cnt++;
        }
    }


    return;
}