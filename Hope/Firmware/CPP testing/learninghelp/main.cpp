#include <iostream>
#include "lib.h"
#include <Eigen/Dense>
using namespace Eigen;

Eigen::Matrix3d stationpositions {
    {-2,1,1},
    {7,5,1},
    {7,-5,1},
};

double stationdistances[4] = {10,10,10};

Eigen::Vector3d location1;

Eigen::Vector3d location2;


int main() {
    // Define a 3x3 matrix where each row is a reference point (x, y, z)
    Matrix3d points;
    points << -2, 4, 1,  // First reference point
              3, -1, 1, // Second reference point
              -5, -4, 1; // Third reference point

    // Define a 1x3 vector of radii
    Vector3d radii(5, 5, 5); // Distances to target point

    Vector3d solution1, solution2;

    if (trilaterate(points, radii, solution1, solution2)) {
        std::cout << "Solution 1: " << solution1.transpose() << "\n";
        std::cout << "Solution 2: " << solution2.transpose() << "\n";
    } else {
        std::cout << "No valid solution found!\n";
    }

    return 0;
}