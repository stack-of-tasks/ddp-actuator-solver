#include "dcmotor.h"
#include <math.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <sys/time.h>
#include <iostream>

DCMotor::DCMotor(double& mydt,bool noiseOnParameters)
{
    stateNb=4;
    commandNb=1;
    dt = mydt;
    struct timeval tv;

    if(!noiseOnParameters)
    {
        R = 1.0;
        L = 0.3e-3;
        Jm = 138e-7;
        fvm = 0.003;
        Kt = 60.3e-3;
    }
    else
    {
        R = 1.0 + 1.0*0.5*(2.0*(rand()/(double)RAND_MAX)-1.0);;
        L = 0.3e-3 + 0.3e-3*0.5*(2.0*(rand()/(double)RAND_MAX)-1.0);;
        Jm = 138e-7 + 138e-7*0.5*(2.0*(rand()/(double)RAND_MAX)-1.0);;
        fvm = 0.003 + 0.003*0.5*(2.0*(rand()/(double)RAND_MAX)-1.0);;
        Kt = 60.3e-3 + 60.3e-3*0.5*(2.0*(rand()/(double)RAND_MAX)-1.0);;
    }

    Id.setIdentity();

    A <<   0.0,1.0,0.0,
            0.0,-fvm/Jm,1.0/Jm,
            0.0,-Kt/L,-R/L;

    Ad = (dt*A).exp();

    B << 0.0,0.0,1.0/L;
    Bd << dt*B;

    fu << 0.0,0.0,1.0/L;
    fu << dt*fu;
    fx.setZero();
    fx = Ad;

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

    lowerCommandBounds << -40.0;
    upperCommandBounds << 40.0;
}

DCMotor::stateVec_t DCMotor::computeStateDeriv(double &dt, const stateVec_t& X, const commandVec_t &U)
{
    stateVec_t x_dot;
    x_dot <<    X(1,0),
                (1.0/Jm)*X(2,0) - (fvm/Jm)*X(1,0),
                (1.0/L)*U(0,0) - (Kt/L)*X(1,0) - (R/L)*X(2,0);
    return x_dot;
}

DCMotor::stateVec_t DCMotor::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    stateVec_t x_next;
    x_next = Ad*X + Bd*U;
    return x_next;
}

void DCMotor::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{

}

DCMotor::stateMat_t DCMotor::computeTensorContxx(const stateVec_t& nextVx)
{
    return QxxCont;
}

DCMotor::commandMat_t DCMotor::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

DCMotor::commandR_stateC_t DCMotor::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}
