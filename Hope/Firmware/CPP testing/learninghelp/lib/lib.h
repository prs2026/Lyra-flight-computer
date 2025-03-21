#pragma once
#include <Eigen/Dense>
using namespace Eigen;

void function2();

void printthenews();

bool trilaterate(const Matrix3d &points, const Vector3d &radii,
    Vector3d &solution1, Vector3d &solution2);


