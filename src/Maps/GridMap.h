/*********************************************************************
 * 
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A implement of GridMap
 * 
*********************************************************************/

#include<vector>
#include<memory>

#define GRIDMAP_IS_OBSTACLE 1
#define GRIDMAP_NOT_OBSTACLE 0

namespace Maps
{
    namespace Utils{
        typedef std::vector<std::vector<int>> MapDataType;
        void GridMapDilateDefaultkernel(MapDataType*,int ,int);
    }
    
    class GridMap{
    public:
        GridMap() = delete;
        GridMap(size_t x, size_t y, double reslotion);
        ~GridMap(){};
        
        size_t getSizeX() const {return x_size_;}
        size_t getSizeY() const {return y_size_;}
        double getMaxX() const {return x_upper_;}
        double getMaxY() const {return y_upper_;}

        void SetObstacleByPos(double const &pt_x, double const &pt_y);
        void SetObstacleByIdx(int const &x, int const &y);
        void ReSetObstacle(unsigned int x, unsigned int y);
        // True for isObs.
        bool HasObstacle(unsigned int x, unsigned int y) const;

        int operator()(int x,int y) const;
        double getReslotion() const {return reslotion_;}

        std::vector<int> ToIdx(double x,double y){return {static_cast<int>(x/reslotion_), static_cast<int>(y/reslotion_)};}
        std::vector<double> ToPosition(int x,int y){return {static_cast<double>((x+0.5)*reslotion_), static_cast<double>((y+0.5)/reslotion_)};}
        
    public :
        typedef std::vector<std::vector<int>> MapDataType;
        void Dilate(std::shared_ptr<GridMap> &DST, void (*kernel)(MapDataType*,int,int)=Utils::GridMapDilateDefaultkernel);
    
    private:
        
        int& operator()(int x,int y);
        MapDataType MapData;
        size_t x_size_,y_size_;
        double reslotion_; // meters per pix
        double x_upper_, y_upper_;
    };
    
} // namespace Planning
