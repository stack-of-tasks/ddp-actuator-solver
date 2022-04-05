#ifndef COSTFUNCTIONPYRENEACTUATOR_H
#define COSTFUNCTIONPYRENEACTUATOR_H

#include <ddp-actuator-solver/costfunction.hh>
#include <vector>

class CostFunctionPyreneActuator : public CostFunction<double, 2, 1> {
 public:
  CostFunctionPyreneActuator();

  void computeCostAndDeriv(const stateVec_t& X, const stateVec_t& Xdes,
                           const commandVec_t& U);
  void computeFinalCostAndDeriv(const stateVec_t& X, const stateVec_t& Xdes);

  void setCostGainState(const stateMat_t& Q);
  void setCostGainStateConstraint(const stateMat_t& W);
  void setCostGainCommand(const commandMat_t& R);
  void setCostGainTorqueConstraint(const commandMat_t& P);

  void setTauLimit(const double& limit);
  void setJointLimit(const double& limitUp, const double& limitDown);
  void setJointVelLimit(const double& limitUp, const double& limitDown);
  void computeConstraintsAndDeriv(const stateVec_t& X);
  void computeTauConstraintsAndDeriv(const commandVec_t& U);

  static const double K;
  static const double offset_m;

 private:
  stateMat_t Q;
  stateMat_t W;
  commandMat_t R;
  commandMat_t P;
  double dt;
  double tauLim;
  double alphaTau;
  double lambdaLimVel;
  double lambdaLimPos;
  std::vector<double> jointLim;
  std::vector<double> jointVelLim;

  stateVec_t Constraints;
  stateMat_t dConstraints;
  stateMat_t ddConstraints;
  commandVec_t TauConstraints;
  commandVec_t dTauConstraints;
  commandVec_t ddTauConstraints;
};

#endif  // COSTFUNCTIONPYRENEACTUATOR_H
