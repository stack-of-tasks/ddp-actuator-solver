#include <math.h>
#include <cmath>  
#include <iostream>
#include "PyreneActuator.hh"
#define pi M_PI

const double PyreneActuator::K = 10.6; 
const double PyreneActuator::J_j = 0.5878;
const double PyreneActuator::F_vj = 0.4757;
const double PyreneActuator::F_sj = 0.5403;
const double PyreneActuator::offset_j = -0.3012;
const double PyreneActuator::J_m = 0.21;
const double PyreneActuator::F_vm = 5.6714;
const double PyreneActuator::F_sm = 4.0420;
const double PyreneActuator::offset_m = 0.5572;
const double PyreneActuator::M = 9.2007;
const double PyreneActuator::c_y = 1.1005/M;
const double PyreneActuator::c_x = -0.0981/M; 
const double PyreneActuator::mu = 1000.0;
const double PyreneActuator::g = -9.81;

/*
 * x0 -> actuator position
 * x1 -> actuator speed
 */

PyreneActuator::PyreneActuator()
{
  stateNb=2;
  commandNb=1;
  J = J_m + J_j;
  F_v = F_vm + F_vj;
  F_s = F_sm + F_sj; 

  L =  0.0;
  l_y = c_y;
  l_x = c_x;

  fu << 0.0, K/J;
  fx << 0.0, 1.0,
        0.0, -F_v/J;

  fxx[0].setZero();
  fxx[1].setZero();

  fxu[0].setZero();
  fux[0].setZero();

  fuu[0].setZero();

  QxxCont.setZero();
  QuuCont.setZero();
  QuxCont.setZero();

  lowerCommandBounds << -1.0;
  upperCommandBounds << 1.0;
}

void PyreneActuator::setLoadParam(const double& mass, const double& coordX, const double& coordY)
{
  L =  mass;
  l_y = coordY;
  l_x = coordX;
  std::cout << " L : " << L << std::endl; 
  std::cout << " l_y : " << l_y << std::endl; 
  std::cout << " l_x : " << l_x << std::endl; 
}

void PyreneActuator::setLoadMass(const double& mass)
{
  L = mass;
  std::cout << " L : " << L << std::endl; 
}

void PyreneActuator::removeLoad()
{
  L = 0.0;
}

PyreneActuator::stateVec_t PyreneActuator::computeStateDeriv(double&, const stateVec_t& X,
    const commandVec_t &U)
{
  stateVec_t dX;

  dX[0] = X[1];
  dX[1] = (1/J) * (K * U[0] - F_v * X[1] - F_s * tanh(mu * X[1])) - (M*g/J) * (cos(X[0])*c_x + sin(X[0])*c_y) \
          - (L*g/J) * (cos(X[0])*l_x + sin(X[0])*l_y) - (offset_m + offset_j);
  return dX;
}

PyreneActuator::stateVec_t PyreneActuator::computeNextState(double& dt, const stateVec_t& X,
    const commandVec_t& U)
{
  stateVec_t x_next,k1,k2,k3,k4;
  k1 = computeStateDeriv(dt, X, U);
  k2 = computeStateDeriv(dt, X + (dt / 2) * k1, U);
  k3 = computeStateDeriv(dt, X + (dt / 2) * k2, U);
  k4 = computeStateDeriv(dt, X + dt * k3, U);
  x_next = X + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
  return x_next;
}


void PyreneActuator::computeModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{
  double dh = 1e-7;
  stateVec_t Xp, Xm;
  Xp = X;
  Xm = X;
  for (unsigned int i = 0; i < stateNb; i++)
  {
    Xp[i] += dh / 2;
    Xm[i] -= dh / 2;
    fx.col(i) = (computeNextState(dt, Xp, U) - computeNextState(dt, Xm, U)) / dh;
    Xp = X;
    Xm = X;
  }
}

PyreneActuator::stateMat_t PyreneActuator::computeTensorContxx(const stateVec_t& )
{
  return QxxCont;
}

PyreneActuator::commandMat_t PyreneActuator::computeTensorContuu(const stateVec_t& )
{
  return QuuCont;
}

PyreneActuator::commandR_stateC_t PyreneActuator::computeTensorContux(const stateVec_t& )
{
  return QuxCont;
}
