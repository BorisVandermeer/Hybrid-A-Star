/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A Simple implement of Vec2D
 * 
*********************************************************************/


#pragma once

#include <iosfwd>
#include <Points/PointsType.h>


namespace Vectors {

    class  Vector2D {
    public:
        Vector2D();
        Vector2D(double _x, double _y){ x = _x, y = _y;}
        Vector2D(Points::PosPoint2D p):x(p.x),y(p.y){};
        Vector2D operator-() const;
        double operator*(const Vector2D &vector) const;
        Vector2D operator*(double scalar) const;
        Vector2D operator/(double scalar) const;
        Vector2D operator+(const Vector2D &vector) const;
        Vector2D operator-(const Vector2D &vector) const;
        bool operator==(const Vector2D &vector) const;
        bool operator!=(const Vector2D &vector) const;
        Vector2D &operator*=(double scalar);
        Vector2D &operator/=(double scalar);
        Vector2D &operator+=(const Vector2D &vector);
        Vector2D &operator-=(const Vector2D &vector);
        
    public:
        double x,y;
    };

    Vector2D operator*(double scalar, const Vector2D &vector);

    std::ostream &operator<<(std::ostream &stream,const Vector2D &vector);

    double abs(const Vector2D &vector);

    double absSq(const Vector2D &vector);

    double det(const Vector2D &vector1, const Vector2D &vector2);

    double leftOf(const Vector2D &vector1, const Vector2D &vector2,const Vector2D &vector3);

    Vector2D normalize(const Vector2D &vector);

    bool isParallel(Vector2D const & v1, Vector2D const & v2);

} // namespace Vectors
