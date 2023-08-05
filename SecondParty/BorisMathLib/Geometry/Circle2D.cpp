/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple implement of 2D Circles
 * 
*********************************************************************/

#include<cmath>
#include<Geometry/Circle2D.h>

#define A_PRIVATE_VALUE (1e-5)

namespace Geometry{
    
    Vectors::Vector2D solveCenterPointOfCircle(Vectors::Vector2D p1,Vectors::Vector2D p2, Vectors::Vector2D p3){
        // Referance :: https://blog.csdn.net/liyuanbhu/article/details/52891868·
        double x1 = p1.x, y1 = p1.y;
        double x2 = p2.x, y2 = p2.y;
        double x3 = p3.x, y3 = p3.y;

        double a = x1 - x2;
        double b = y1 - y2;
        double c = x1 - x3;
        double d = y1 - y3;
        double e = ((x1 * x1 - x2 * x2) + (y1 * y1 - y2 * y2)) / 2.0;
        double f = ((x1 * x1 - x3 * x3) + (y1 * y1 - y3 * y3)) / 2.0;
        double det = b * c - a * d;

        if( fabs(det) < A_PRIVATE_VALUE){
            return Vectors::Vector2D(NAN,NAN);
        }

        double x0 = -(d * e - b * f) / det;
        double y0 = -(a * f - c * e) / det;
        return Vectors::Vector2D(x0,y0);

    }

}