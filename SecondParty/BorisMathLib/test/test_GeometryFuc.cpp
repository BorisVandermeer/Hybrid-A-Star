
#include<iostream>
#include<cmath>
#include<Geometry/Line2D.h>
#include<Geometry/Circle2D.h>

using namespace std;
using Vectors::Vector2D;

int main(){
    // Test GetIntersection
    {
        Geometry::Line l1;
        Geometry::Line l2;
        Vector2D Intersection(3,5);
        double phi1 = 10,k1 = 10;
        double phi2 = 15,k2 = 40;;
        l1.point = Intersection + k1*Vector2D(cos(phi1),sin(phi1));
        l1.direction = Vector2D(cos(phi1),sin(phi1));
        l2.point = Intersection + k2*Vector2D(cos(phi2),sin(phi2));
        l2.direction = Vector2D(cos(phi2),sin(phi2));
        std::cout<<Vector2D(Geometry::GetIntersection(l1,l2))<<std::endl;
    }

    // Test solveCenterPointOfCircle
    {
        Vector2D center(5,6);
        double phi1 = -10,phi2 = 15,phi3 = 60;
        double R = 100;
        Vector2D p1 = center + R*Vector2D(cos(phi1),sin(phi1));
        Vector2D p2 = center + R*Vector2D(cos(phi2),sin(phi2));
        Vector2D p3 = center + R*Vector2D(cos(phi3),sin(phi3));

        std::cout<<Geometry::solveCenterPointOfCircle(p1,p2,p3)<<std::endl;
    }

    return 0;
}