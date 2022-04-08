#include "ddp-actuator-solver/pyrene_actuator/pyreneActuator.hh"

#include <math.h>

#include <cmath>
#include <iostream>
#define pi M_PI

const double pyreneActuator::K = 10.6;
const double pyreneActuator::J_j = 0.5878;
const double pyreneActuator::F_vj = 0.4757;
const double pyreneActuator::F_sj = 0.5403;
const double pyreneActuator::offset_j = -0.3012;
const double pyreneActuator::J_m = 0.21;
const double pyreneActuator::F_vm = 5.6714;
const double pyreneActuator::F_sm = 4.0420;
const double pyreneActuator::offset_m = 0.5572;
const double pyreneActuator::M = 4.5667;
const double pyreneActuator::c_y = 1.1005 / M;
const double pyreneActuator::c_x = -0.0981 / M;
const double pyreneActuator::mu = 1000.0;
const double pyreneActuator::g = 9.81;

/*
 * x0 -> actuator position
 * x1 -> actuator speed
 */

pyreneActuator::pyreneActuator() {
  stateNb = 2;
  commandNb = 1;
  J = J_m + J_j;
  F_v = F_vm + F_vj;
  F_s = F_sm + F_sj;

  L = 0.0;
  l_y = c_y;
  l_x = c_x;

  fu << 0.0, K / J;
  fx.setZero();

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

void pyreneActuator::setLoadParam(const double& mass, const double& coordX,
                                  const double& coordY) {
  L = mass;
  l_y = coordY;
  l_x = coordX;
}

void pyreneActuator::setLoadMass(const double& mass) {
  L = mass;
  std::cout << " L : " << L << std::endl;
}

void pyreneActuator::removeLoad() { L = 0.0; }

pyreneActuator::stateVec_t pyreneActuator::computeStateDeriv(
    double&, const stateVec_t& X, const commandVec_t& U) {
  stateVec_t dX;

  dX[0] = X[1];
  dX[1] = (1 / J) * (K * U[0] - F_v * X[1] - F_s * tanh(mu * X[1])) -
          (M * g / J) * (cos(X[0]) * c_x + sin(X[0]) * c_y) -
          (L * g / J) * (cos(X[0]) * l_x + sin(X[0]) * l_y) -
          (offset_m + offset_j);
  return dX;
}

pyreneActuator::stateVec_t pyreneActuator::computeNextState(
    double& dt, const stateVec_t& X, const commandVec_t& U) {
  stateVec_t x_next, k1, k2, k3, k4;
  k1 = computeStateDeriv(dt, X, U);
  k2 = computeStateDeriv(dt, X + (dt / 2) * k1, U);
  k3 = computeStateDeriv(dt, X + (dt / 2) * k2, U);
  k4 = computeStateDeriv(dt, X + dt * k3, U);
  x_next = X + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
  return x_next;
}

void pyreneActuator::computeModelDeriv(double& dt, const stateVec_t& X,
                                       const commandVec_t& U) {
  double dh = 1e-7;
  stateVec_t Xp, Xm;
  Xp = X;
  Xm = X;
  for (unsigned int i = 0; i < stateNb; i++) {
    Xp[i] += dh / 2;
    Xm[i] -= dh / 2;
    fx.col(i) =
        (computeNextState(dt, Xp, U) - computeNextState(dt, Xm, U)) / dh;
    Xp = X;
    Xm = X;
  }
}

pyreneActuator::stateMat_t pyreneActuator::computeTensorContxx(
    const stateVec_t&) {
  return QxxCont;
}

pyreneActuator::commandMat_t pyreneActuator::computeTensorContuu(
    const stateVec_t&) {
  return QuuCont;
}

pyreneActuator::commandR_stateC_t pyreneActuator::computeTensorContux(
    const stateVec_t&) {
  return QuxCont;
}
