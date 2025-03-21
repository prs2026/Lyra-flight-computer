#include <iostream>
#include "lib.h"

using namespace Eigen;


void function2(){
    std::cout << "are we so back"<<"\n";
    return;
}

void printthenews(){
    std::cout << "are we so back"<<"\n";
    return;
}

bool trilaterate(const Matrix3d &points, const Vector3d &radii,
    Vector3d &solution1, Vector3d &solution2) {

    // Extract the three reference points
    Vector3d p1 = points.row(0);
    Vector3d p2 = points.row(1);
    Vector3d p3 = points.row(2);

    // Extract the radii
    double r1 = radii(0);
    double r2 = radii(1);
    double r3 = radii(2);

    // Compute unit vector ex (direction from p1 to p2)
    Vector3d ex = p2 - p1;
    double d = ex.norm();  // Distance between p1 and p2
    if (d == 0) return false;  // Prevent division by zero
    ex.normalize();

    // Compute the vector from p1 to p3
    Vector3d temp = p3 - p1;

    // Compute i as dot product temp ⋅ ex
    double i = temp.dot(ex);

    // Compute unit vector ey (perpendicular to ex in the plane of p1, p2, p3)
    Vector3d ey = temp - i * ex;
    double j = ey.norm();
    if (j == 0) return false;  // Prevent division by zero
    ey.normalize();

    // Compute unit vector ez (perpendicular to the plane formed by p1, p2, p3)
    Vector3d ez = ex.cross(ey);

    // Solve for x, y
    double x = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    double y = (r1 * r1 - r3 * r3 + i * i + j * j - 2 * i * x) / (2 * j);
    double zSquared = r1 * r1 - x * x - y * y;

    if (zSquared < 0) return false;  // No real solution

    double z1 = std::sqrt(zSquared);
    double z2 = -z1;  // Second solution

    // Compute both possible solutions in original space
    solution1 = p1 + x * ex + y * ey + z1 * ez;
    solution2 = p1 + x * ex + y * ey + z2 * ez;

    return true;
}

