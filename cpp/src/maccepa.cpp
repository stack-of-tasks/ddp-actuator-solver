#include "maccepa.h"
#include <math.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <sys/time.h>
#include <iostream>

#define pi M_PI

/*
 * x0 -> actuator position
 * x1 -> spring torque
 * x2 -> actuator speed
 * x3 -> motor speed
 */

Maccepa::Maccepa(double& mydt)
{
    stateNb=4;
    commandNb=1;
    dt = mydt;
    struct timeval tv;

    k = 2520;
    //B = 0.01;
    //C = 0.04;
    //P = 0.04;

    Id.setIdentity();

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();

    fxu[0].setZero();
    fxu[0].setZero();

    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    QxxCont.setZero();
    QuuCont.setZero();
    QuxCont.setZero();

    lowerCommandBounds << -5.0;
    upperCommandBounds << 5.0;
}

Maccepa::stateVec_t Maccepa::computeStateDeriv(double &dt, const stateVec_t& X, const commandVec_t &U)
{
    stateVec_t x_dot;
    x_dot << X(1,0);

    return x_dot;
}

Maccepa::stateVec_t Maccepa::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    stateVec_t x_next,k1,k2,k3,k4;
    /*k1 = A*X + B*U;
    k2 = A*(X+(dt/2)*k1) + B*U;
    k3 = A*(X+(dt/2)*k2) + B*U;
    k4 = A*(X+dt*k3) + B*U;
    x_next = X + (dt/6)*(k1+2*k2+2*k3+k4);*/

    k1 = computeStateDeriv(dt,X,U);
    k2 = computeStateDeriv(dt,X+(dt/2.0)*k1,U);
    k3 = computeStateDeriv(dt,X+(dt/2.0)*k2,U);
    k4 = computeStateDeriv(dt,X+dt*k3,U);

    x_next = X + (dt/6.0)*(k1+2.0*k2+2.0*k3+k4);
    //x_next = Ad*X + Bd*U;
    return x_next;
}

void Maccepa::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    double dh = 1e-7;
    stateVec_t Xp,Xm;
    Xp = X;
    Xm = X;
    for(int i=0;i<4;i++)
    {
        Xp[i] += dh/2;
        Xm[i] -= dh/2;
        fx.col(i) = (computeNextState(dt, Xp, U) - computeNextState(dt, Xm, U))/dh;
        Xp = X;
        Xm = X;
    }
}

Maccepa::stateMat_t Maccepa::computeTensorContxx(const stateVec_t& nextVx)
{
    return QxxCont;
}

Maccepa::commandMat_t Maccepa::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

Maccepa::commandR_stateC_t Maccepa::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}
