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
const double DIFF_EPS = 1e-6;      
const double SINGULAR_EPS = 1e-12;  

class MechanismSolver {
public:
    enum class Side { Left, Right };

    MechanismSolver(double _l1, double _d1, double _h1, double _h2, Side side);

    bool forwardKinematics(double t1, double t2, double& theta_p, double& theta_r);

    bool inverseKinematics(double theta_p, double theta_r, double& t1, double& t2);

    bool motorVelToJointVel(double t1, double t2, double tp, double tr,
                            double t1dot, double t2dot,
                            double& tpdot, double& trdot);

    void resetWarmStart();

private:
    Side side_;

    double l1, d1, h1, h2;

    bool   has_last_fk_{false};
    double last_tp_{0.0};
    double last_tr_{0.0};

    bool   has_last_ik_{false};
    double last_t1_{0.0};
    double last_t2_{0.0};

    double distSq(const Point3D& p1, const Point3D& p2);

    void getPoints(double t1, double t2, double tp, double tr, 
                   Point3D& A, Point3D& B, Point3D& C, Point3D& D);

    bool solveNewton(bool is_forward, 
                     double known_1, double known_2,
                     double& out_1, double& out_2,
                     double x1_init, double x2_init);

    void constraints(double t1, double t2, double tp, double tr,
                     double& f1, double& f2);

    bool constraintJacobians(double t1, double t2, double tp, double tr,
                             double& Gm11, double& Gm12, double& Gm21, double& Gm22,
                             double& Gj11, double& Gj12, double& Gj21, double& Gj22);

    static bool solve2x2(double a11, double a12, double a21, double a22,
                         double b1, double b2,
                         double& x1, double& x2);

};

#endif // MECHANISM_SOLVER_H

