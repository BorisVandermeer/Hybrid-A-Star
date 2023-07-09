/*********************************************************************
 * 
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A implement of GridMap
 * 
*********************************************************************/

#include<assert.h>
#include<vector>
#include<functional>

#include<GridMap.h>

using namespace std;

namespace Maps{

    GridMap::GridMap(size_t x, size_t y, double reslotion)
    {
        assert((x>0)&&(y>0));
        std::vector<int> tmp(y,GRIDMAP_NOT_OBSTACLE);
        MapData.resize(x,tmp);
        x_size_ = x;y_size_ = y;
        x_upper_ = x*reslotion;
        y_upper_ = y*reslotion; 
        reslotion_ = reslotion;
    }

    void GridMap::SetObstacleByIdx(int const &x, int const &y)
    {
        MapData[x][y] = GRIDMAP_IS_OBSTACLE;
    }

    void GridMap::SetObstacleByPos(double const &x, double const &y)
    {
        unsigned int idx_x = x/reslotion_;
        unsigned int idx_y = y/reslotion_;
        SetObstacleByIdx(idx_x, idx_y);
    }

    void GridMap::ReSetObstacle(unsigned int x, unsigned int y)
    {
        MapData[x][y] = GRIDMAP_NOT_OBSTACLE;
    }
    

    bool GridMap::HasObstacle(unsigned int x, unsigned int y) const
    {
        if(MapData[x][y] == GRIDMAP_IS_OBSTACLE) return true;
        return false;
    }
    
    int GridMap::operator()(int x,int y) const
    {
        return MapData[x][y];
    }

    int& GridMap::operator()(int x,int y)
    {
        return MapData[x][y];
    }

    void Utils::GridMapDilateDefaultkernel(GridMap::MapDataType *pData,int x, int y){
        int size_x = pData->size();
        int size_y = pData->front().size();

        if(x==0&&y==0){ 
            (*pData)[x  ][y  ] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y  ] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x  ][y+1] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y+1] = GRIDMAP_IS_OBSTACLE;
        } 
        else if(x==size_x-1&&y==0){
            (*pData)[x-1][y  ] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y  ] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x-1][y+1] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y+1] = GRIDMAP_IS_OBSTACLE; 
        }
        else if(x==0&&y==size_y-1){
            (*pData)[x  ][y-1] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y-1] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x  ][y  ] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y  ] = GRIDMAP_IS_OBSTACLE;
        }
        else if(x==size_x-1&&y==size_y-1){
            (*pData)[x-1][y-1] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y-1] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x-1][y  ] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y  ] = GRIDMAP_IS_OBSTACLE;
        }
        else if(x==0){
            (*pData)[x  ][y-1] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y-1] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x  ][y  ] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y  ] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x  ][y+1] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y+1] = GRIDMAP_IS_OBSTACLE;
        }
        else if(x==size_x-1){
            (*pData)[x-1][y-1] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y-1] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x-1][y  ] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y  ] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x-1][y+1] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y+1] = GRIDMAP_IS_OBSTACLE;
        }
        else if(y==0){
            (*pData)[x-1][y  ] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y  ] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y  ] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x-1][y+1] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y+1] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y+1] = GRIDMAP_IS_OBSTACLE;
        }
        else if(y==size_y-1){
            (*pData)[x-1][y-1] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y-1] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y-1] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x-1][y  ] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y  ] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y  ] = GRIDMAP_IS_OBSTACLE;
        }
        else {
            (*pData)[x-1][y-1] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y-1] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y-1] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x-1][y  ] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y  ] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y  ] = GRIDMAP_IS_OBSTACLE;
            (*pData)[x-1][y+1] = GRIDMAP_IS_OBSTACLE; (*pData)[x  ][y+1] = GRIDMAP_IS_OBSTACLE;  (*pData)[x+1][y+1] = GRIDMAP_IS_OBSTACLE;
        }

    }

    void GridMap::Dilate(shared_ptr<GridMap> &DST, void (*kernel)(MapDataType*,int,int)){
        DST = make_shared<GridMap>(x_size_,y_size_,reslotion_);
        for(size_t x=0;x<x_size_;x++) for(size_t y=0;y<y_size_;y++){
            if(HasObstacle(x,y)) 
                kernel(&(DST->MapData),x,y);
        }
    }


    // DistMap::DistMap(shared_ptr<GridMap> _pGridMap, double max_dist){
    //     pGridMap = _pGridMap;
    //     x_size_ = pGridMap->getSizeX();
    //     y_size_ = pGridMap->getSizeY();
    //     x_upper_ = pGridMap->getMaxX();
    //     y_upper_ = pGridMap->getMaxY();
    //     reslotion_ = pGridMap->getReslotion();
    //     MapData.clear();
    //     MapData.resize(x_size_*y_size_, __DBL_MAX__);
        
    // }

}// namespace Planning