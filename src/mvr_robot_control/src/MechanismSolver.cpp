//-----------------------------------------------------------------------------
// File: MechanismSolver.cpp
// Author: Akisora
// Created: 2025-12-17
//-----------------------------------------------------------------------------

#include "mvr_robot_control/MechanismSolver.h"
#include <cmath> 
#include <iostream>

MechanismSolver::MechanismSolver(double _l1, double _d1, double _h1, double _h2)
    : l1(_l1), d1(_d1), h1(_h1), h2(_h2) {

}

double MechanismSolver::distSq(const Point3D& p1, const Point3D& p2) {
    return std::pow(p1.x - p2.x, 2) + 
           std::pow(p1.y - p2.y, 2) + 
           std::pow(p1.z - p2.z, 2);
}

void MechanismSolver::getPoints(double t1, double t2, double tp, double tr, 
                   Point3D& A, Point3D& B, Point3D& C, Point3D& D) {

    double c1 = cos(t1), s1 = sin(t1);
    double c2 = cos(t2), s2 = sin(t2);
    double cp = cos(tp), sp = sin(tp);
    double cr = cos(tr), sr = sin(tr);

    A = { l1, d1 * c1, h1 + d1 * s1 };
    B = { l1, -d1 * c2, h2 - d1 * s2 };
    C = { l1 * cp, d1 * cr, l1 * sp + d1 * sr };
    D = { l1 * cp, -d1 * cr, l1 * sp - d1 * sr };
}

bool MechanismSolver::solveNewton(bool is_forward, double known_1, double known_2, double& out_1, double& out_2) {
    double x1 = 0.0; 
    double x2 = 0.0; 

    for (int i = 0; i < MAX_ITER; ++i) {
        Point3D A, B, C, D;
        
        if (is_forward) {
            getPoints(known_1, known_2, x1, x2, A, B, C, D);
        } else {
            getPoints(x1, x2, known_1, known_2, A, B, C, D);
        }

        double f1 = distSq(A, C) - h1 * h1;
        double f2 = distSq(B, D) - h2 * h2;

        if (std::abs(f1) < EPSILON && std::abs(f2) < EPSILON) {
            out_1 = x1;
            out_2 = x2;
            return true; 
        }

        double delta = 1e-5;
        Point3D A_d, B_d, C_d, D_d;

        double f1_dx1, f2_dx1;
        if (is_forward) getPoints(known_1, known_2, x1 + delta, x2, A_d, B_d, C_d, D_d);
        else            getPoints(x1 + delta, x2, known_1, known_2, A_d, B_d, C_d, D_d);
        
        f1_dx1 = (distSq(A_d, C_d) - h1 * h1 - f1) / delta;
        f2_dx1 = (distSq(B_d, D_d) - h2 * h2 - f2) / delta;

        double f1_dx2, f2_dx2;
        if (is_forward) getPoints(known_1, known_2, x1, x2 + delta, A_d, B_d, C_d, D_d);
        else            getPoints(x1, x2 + delta, known_1, known_2, A_d, B_d, C_d, D_d);

        f1_dx2 = (distSq(A_d, C_d) - h1 * h1 - f1) / delta;
        f2_dx2 = (distSq(B_d, D_d) - h2 * h2 - f2) / delta;

        double det = f1_dx1 * f2_dx2 - f1_dx2 * f2_dx1;

        if (std::abs(det) < 1e-9) {
            std::cerr << "Error: Jacobian determinant is zero (Singularity)." << std::endl;
            return false;
        }

        double d_x1 = ((-f1) * f2_dx2 - (-f2) * f1_dx2) / det;
        double d_x2 = (f1_dx1 * (-f2) - f2_dx1 * (-f1)) / det;

        x1 += d_x1;
        x2 += d_x2;
    }

    return false; 
}

bool MechanismSolver::forwardKinematics(double t1, double t2, double& theta_p, double& theta_r) {
    return solveNewton(true, t1, t2, theta_p, theta_r);
}

bool MechanismSolver::inverseKinematics(double theta_p, double theta_r, double& t1, double& t2) {
    return solveNewton(false, theta_p, theta_r, t1, t2);
}

