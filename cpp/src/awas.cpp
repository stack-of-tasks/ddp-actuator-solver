#include "awas.h"
#include <math.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <sys/time.h>

#define pi M_PI

/*
 *
 */
Awas::Awas(double& mydt,bool noiseOnParameters)
{
    stateNb=4;
    commandNb=2;
    dt = mydt;
    struct timeval tv;

    if(!noiseOnParameters)
    {
        k = 10000.0;
        M = 0.018;
    }
    else
    {
        gettimeofday(&tv,NULL);
        srand(tv.tv_usec);
        k = 10000.0 + 10000.0*0.1*(2.0*(rand()/(double)RAND_MAX)-1.0);
        M = 0.018 + 0.018*0.1*(2.0*(rand()/(double)RAND_MAX)-1.0);
    }

    Id.setIdentity();

    A.setZero();
    Ad = (dt*A).exp();

    B <<    0.0,0.0,
            1.0,0.0,
            0.0,0.0,
            0.0,1.0;
    Bd = dt*B;

    fu = Bd;

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();

    fxu[0].setZero();
    fxu[0].setZero();

    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    QxxCont.setZero();
    QuuCont.setZero();
    QuxCont.setZero();

    lowerCommandBounds << 0.01;
    upperCommandBounds << 0.1;
}

Awas::stateVec_t Awas::computeStateDeriv(double &dt, const stateVec_t& X, const commandVec_t &U)
{
    stateVec_t x_dot;
    x_dot <<    X(1,0),
                U(0,0) - (k/M)*X(2,0)*X(2,0)*sin(X(0,0)),
                X(3,0),
                U(1,0);
    return x_dot;
}

Awas::stateVec_t Awas::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    stateVec_t x_next,k1,k2,k3,k4;

    k1 = computeStateDeriv(dt,X,U);
    k2 = computeStateDeriv(dt,X+(dt/2.0)*k1,U);
    k3 = computeStateDeriv(dt,X+(dt/2.0)*k2,U);
    k4 = computeStateDeriv(dt,X+dt*k3,U);

    x_next = X + (dt/6.0)*(k1+2.0*k2+2.0*k3+k4);
    return x_next;
}

void Awas::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    double dh = 1e-7;
    stateVec_t Xp,Xm;
    Xp = X;
    Xm = X;
    for(int i=0;i<4;i++)
    {
        Xp[i] += dh/2;
        Xm[i] -= dh/2;
        fx.col(i) = (computeNextState(dh, Xp, U) - computeNextState(dh, Xm, U))/dh;
        Xp = X;
        Xm = X;
    }
}

Awas::stateMat_t Awas::computeTensorContxx(const stateVec_t& nextVx)
{
    return QxxCont;
}

Awas::commandMat_t Awas::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

Awas::commandR_stateC_t Awas::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}
