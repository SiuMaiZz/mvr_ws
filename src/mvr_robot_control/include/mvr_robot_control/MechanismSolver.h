//-----------------------------------------------------------------------------
// File: MechanismSolver.h
// Author: Akisora
// Created: 2025-12-17
//-----------------------------------------------------------------------------

#ifndef MECHANISM_SOLVER_H
#define MECHANISM_SOLVER_H

#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>

struct Point3D {
    double x, y, z;
};

const double EPSILON = 1e-6;
const int MAX_ITER = 100;

class MechanismSolver {
public:
    MechanismSolver(double _l1, double _d1, double _h1, double _h2);

    bool forwardKinematics(double t1, double t2, double& theta_p, double& theta_r);

    bool inverseKinematics(double theta_p, double theta_r, double& t1, double& t2);

private:
    double l1, d1, h1, h2;

    double distSq(const Point3D& p1, const Point3D& p2);

    void getPoints(double t1, double t2, double tp, double tr, 
                   Point3D& A, Point3D& B, Point3D& C, Point3D& D);

    bool solveNewton(bool is_forward, double known_1, double known_2, double& out_1, double& out_2);

};

#endif // MECHANISM_SOLVER_H

