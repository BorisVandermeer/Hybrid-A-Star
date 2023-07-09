/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      Describe common Points types
 *      To disable These Types, use " #define _BORIS_POINTS_TYPE_DISABLE_ " 
 *    before where the header is included. Use " #ifdef _BORIS_POINTS_TYPE_ENABLED_ "
 *    to check whether the types here is enabled.
 * 
 * 
 * 
*********************************************************************/
#ifndef _BORIS_POINTS_TYPE_DEF_H_
#define _BORIS_POINTS_TYPE_DEF_H_

#ifndef _BORIS_POINTS_TYPE_DISABLE_
    #define _BORIS_POINTS_TYPE_ENABLED_

#include<vector>

namespace Points{
    template<class DataType = double>
    struct Point2d{ 
        DataType x,y;
        Point2d() = default;
        Point2d(DataType _x, DataType _y):x(_x),y(_y){}
    };

    template<class DataType = double>
    struct Point2ds{ 
        std::vector<DataType> x,y;
        Point2ds() = default;
        size_t size(){return x.size();}
        void clear(){x.clear();y.clear();}
        void push_back(Point2d<DataType> _pos){x.push_back(_pos.x);y.push_back(_pos.y);}
        void set(int idx, Point2d<DataType> _pos){x[idx]=_pos.x,y[idx]=_pos.y;}
        Point2ds<DataType> operator [](int idx){return Point2ds<DataType>(x[idx],y[idx]);}
    };

    template<class DataType = double>
    struct Point2dWithDir{ 
        DataType x,y,phi;
        Point2dWithDir() = default;
        Point2dWithDir(DataType _x, DataType _y, DataType _phi):x(_x),y(_y),phi(_phi){}
    };

    template<class DataType = double>
    struct Point2dWithDirs{ 
        std::vector<DataType> x,y,phi;
        Point2dWithDirs() = default;
        size_t size(){return x.size();}
        void clear(){x.clear();y.clear();phi.clear();};
        void push_back(Point2dWithDir<DataType> _pos){x.push_back(_pos.x);y.push_back(_pos.y);phi.push_back(_pos.phi);}
        void set(int idx, Point2dWithDir<DataType> _pos){x[idx]=_pos.x,y[idx]=_pos.y,phi[idx]=_pos.phi;}
        Point2dWithDir<DataType> operator [](int idx){return Point2dWithDir<DataType>(x[idx],y[idx],phi[idx]);}
    };

    // template<class DataType = double>
    // struct Point2dWithHeading{ DataType x,y,heading;};    

    template<class DataType = double>
    struct Point3d{
        DataType x,y,z;
        Point3d() = default;
        Point3d(DataType _x, DataType _y, DataType _z):x(_x),y(_y),z(_z){}
    };

    template<class DataType = double>
    struct Point3dWithDir{ 
        DataType x,y,z,phi;
        Point3dWithDir() = default;
        Point3dWithDir(DataType _x, DataType _y, DataType _z,DataType _phi):x(_x),y(_y),z(_z),phi(_phi){}
    };

    typedef Point2d<int>    GridPoint2D;
    typedef Point2d<double> PosPoint2D;

    typedef Point2ds<int>    GridPoint2Ds;
    typedef Point2ds<double> PosPoint2Ds;

    typedef Point2dWithDir<int> GridPos2D;
    typedef Point2dWithDir<double> Pos2D;

    typedef Point2dWithDirs<int> GridPos2Ds;
    typedef Point2dWithDirs<double> Pos2Ds;

    typedef Point3d<int>     GridPoint3D;
    typedef Point3d<double>  PosPoint3D;

} // namespace Points

#endif
#endif
