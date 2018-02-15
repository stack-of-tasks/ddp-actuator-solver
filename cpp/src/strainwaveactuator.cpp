#include "strainwaveactuator.h"
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

StrainWaveActuator::StrainWaveActuator(double& mydt)
{
    stateNb=2;
    commandNb=1;
    dt = mydt;
    struct timeval tv;

    Kt = 0.0724015;
    Kv = 0.381202;
    Kf = 0.635621;
    Ka = 0.0;
    a = 10.0;
    J = 0.85;

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

StrainWaveActuator::stateVec_t StrainWaveActuator::computeStateDeriv(double &dt, const stateVec_t& X, const commandVec_t &U)
{
    stateVec_t x_dot;
    if(abs((1.0 / J)*(Kf/Kt)*(2.0/M_PI)*atan(a*X(3,0)))> abs((1.0/(J*Kt))*U(0,0)-(1.0/J)*((Kv/Kt)*X(1,0))))
    {
        x_dot <<    X(1,0),
                0.0;
    }
    else
    {
        x_dot << X(1, 0),
                (1.0 / (J * Kt)) * U(0, 0) -
                (1.0 / J) * (Kv / Kt) * X(1, 0)
            /*+ (1.0 / J) (Kf / Kt) * (2.0 / M_PI) * atan(a * X(3, 0)))*/;
    }

    return x_dot;
}

StrainWaveActuator::stateVec_t StrainWaveActuator::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
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

void StrainWaveActuator::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
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

StrainWaveActuator::stateMat_t StrainWaveActuator::computeTensorContxx(const stateVec_t& nextVx)
{
    return QxxCont;
}

StrainWaveActuator::commandMat_t StrainWaveActuator::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

StrainWaveActuator::commandR_stateC_t StrainWaveActuator::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}
