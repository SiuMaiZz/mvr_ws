//-----------------------------------------------------------------------------
// File: MechanismSolver.cpp
// Author: Akisora
// Created: 2025-12-17
//-----------------------------------------------------------------------------

#include "mvr_robot_control/MechanismSolver.h"
#include <cmath> 
#include <iostream>

MechanismSolver::MechanismSolver(double _l1, double _d1, double _h1, double _h2, Side _side)
    : l1(_l1), d1(_d1), h1(_h1), h2(_h2), side_(_side) {

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

    if (side_ == Side::Left) {
        A = { l1, d1 * c1, h1 + d1 * s1 };
        B = { l1, -d1 * c2, h2 - d1 * s2 };
        C = { l1 * cp, d1 * cr, l1 * sp - d1 * sr };
        D = { l1 * cp, -d1 * cr, l1 * sp + d1 * sr };
    } else {
        A = { l1, -d1 * c1,  h1 - d1 * s1 };
        B = { l1,  d1 * c2,  h2 + d1 * s2 };
        C = { l1 * cp, -d1 * cr, -l1 * sp - d1 * sr };
        D = { l1 * cp,  d1 * cr, -l1 * sp + d1 * sr };
    }
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

void MechanismSolver::constraints(double t1, double t2, double tp, double tr,
                                  double& f1, double& f2)
{
    Point3D A, B, C, D;
    getPoints(t1, t2, tp, tr, A, B, C, D);
    f1 = distSq(A, C) - h1 * h1;   
    f2 = distSq(B, D) - h2 * h2; 
}

bool MechanismSolver::solve2x2(double a11, double a12, double a21, double a22,
                               double b1, double b2,
                               double& x1, double& x2)
{
    double det = a11 * a22 - a12 * a21;
    if (std::abs(det) < SINGULAR_EPS) return false;
    x1 = ( b1 * a22 - a12 * b2) / det;
    x2 = ( a11 * b2 - b1 * a21) / det;
    return true;
}

bool MechanismSolver::constraintJacobians(double t1, double t2, double tp, double tr,
                                          double& Gm11, double& Gm12, double& Gm21, double& Gm22,
                                          double& Gj11, double& Gj12, double& Gj21, double& Gj22)
{
    double f1, f2;
    constraints(t1, t2, tp, tr, f1, f2);

    const double h = DIFF_EPS;
    double f1p, f2p;

    constraints(t1 + h, t2, tp, tr, f1p, f2p);
    Gm11 = (f1p - f1) / h;   
    Gm21 = (f2p - f2) / h;

    constraints(t1, t2 + h, tp, tr, f1p, f2p);
    Gm12 = (f1p - f1) / h;
    Gm22 = (f2p - f2) / h;  

    constraints(t1, t2, tp + h, tr, f1p, f2p);
    Gj11 = (f1p - f1) / h;  
    Gj21 = (f2p - f2) / h;

    constraints(t1, t2, tp, tr + h, f1p, f2p);
    Gj12 = (f1p - f1) / h; 
    Gj22 = (f2p - f2) / h;   

    double detGj = Gj11 * Gj22 - Gj12 * Gj21;
    if (std::abs(detGj) < SINGULAR_EPS) return false;

    return true;
}

bool MechanismSolver::motorVelToJointVel(double t1, double t2, double tp, double tr,
                                         double t1dot, double t2dot,
                                         double& tpdot, double& trdot)
{
    double Gm11, Gm12, Gm21, Gm22;
    double Gj11, Gj12, Gj21, Gj22;
    if (!constraintJacobians(t1, t2, tp, tr,
                             Gm11, Gm12, Gm21, Gm22,
                             Gj11, Gj12, Gj21, Gj22))
    {
        return false;
    }

    double u1 = Gm11 * t1dot + Gm12 * t2dot;
    double u2 = Gm21 * t1dot + Gm22 * t2dot;

    return solve2x2(Gj11, Gj12, Gj21, Gj22, -u1, -u2, tpdot, trdot);
}