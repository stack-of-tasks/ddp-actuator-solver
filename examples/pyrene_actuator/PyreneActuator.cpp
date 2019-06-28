#include <math.h>
#include <cmath>  
#include <iostream>
#include "PyreneActuator.hh"
#define pi M_PI

const double PyreneActuator::J = 1.4331;
const double PyreneActuator::K = 10.6; 
const double PyreneActuator::F_v = 0.0337;
const double PyreneActuator::F_s = 0.9089;
const double PyreneActuator::M =  9.2007e3;
const double PyreneActuator::L =  20.0e3;
const double PyreneActuator::c_y = -0.0549/M; //TODO /M
const double PyreneActuator::c_x = 3.0835/M; //TODO /M
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

  Id.setIdentity();

  fu << 0.0, K/J;
  fx << 0.0, 1.0,
        0.0, -F_v/J;
  // fx.setZero();
  // fu.setZero();

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


PyreneActuator::stateVec_t PyreneActuator::computeStateDeriv(double&, const stateVec_t& X,
    const commandVec_t &U)
{
  stateVec_t dX;
  dX[0] = X[1];
  dX[1] = ((1/J) * (K * U[0] - F_v * X[1] - F_s * tanh(mu * X[1])) - ((M+L)*g/J) * (cos(X[0])*c_x + sin(X[0])*c_y)); //TODO /M & TAU
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
    // std::cout << computeNextState(dt, Xp, U) - computeNextState(dt, Xm, U) << std::endl;
    Xp = X;
    Xm = X;
  }
  // std::cout << "fx" << std::endl;
  // std::cout << fx << std::endl;
  
  // stateVec_t dX;
  // // dX = computeStateDeriv(dt, X, U);
  // std::cout << "fx" << std::endl;
  // std::cout << fx << std::endl;
  // std::cout << "X0" << std::endl;
  // std::cout << X[0] << std::endl;
  // fx(1,0) = (g/J) * (-cos(X[0])*c_y + sin(X[0])*c_x);
  // std::cout << "fx" << std::endl;
  // std::cout << fx << std::endl;
  
  // stateMat_t matFxx;
  // // std::cout << "matFxx" << std::endl;
  // matFxx << (g/J) * (sin(X[0])*c_y + cos(X[0])*c_x), 0.0,
  //           0.0, 0.0;
  // // // std::cout << matFxx << std::endl;
  // fxx[1] = matFxx;
  // // std::cout << fxx[0] << std::endl;
  // std::cout << fxx[1] << std::endl;
  // fx(1) = (1/J) * (-F_v); //TODO F_s
  // fu(1) = (1/J) * K;
  
  // fxx(1,0) = 0.0;//TODO F_s

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
