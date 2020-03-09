#include "pyreneCostFunction.hh"
#include <iostream>
#include <math.h>
#include <cmath>     

using namespace std;   

const double CostFunctionPyreneActuator::K = 10.6;
const double CostFunctionPyreneActuator::offset_m = 0.5572; 

CostFunctionPyreneActuator::CostFunctionPyreneActuator()
{
    Q << 500.0, 0.0,
         0.0, 0.01; 
    W << 1.0, 0.0,
         0.0, 0.01; 
    R << 0.0001;
    P << 100.0;

    lxx = Q;
    luu = R;
    lux << 0.0,0.0;
    lxu << 0.0,0.0;
    lx.setZero();
    final_cost = 0.0;
    running_cost = 0.0;
    tauLim = 0.0;
    lambdaLimPos = 10.0;
    lambdaLimVel = 1.0;
    alphaTau = 0.5;
}


void CostFunctionPyreneActuator::setCostGainState(const stateMat_t& Q_new)
{
    Q = Q_new;
}

void CostFunctionPyreneActuator::setCostGainStateConstraint(const stateMat_t& W_new)
{
    W = W_new;
}

void CostFunctionPyreneActuator::setCostGainCommand(const commandMat_t& R_new)
{
    R = R_new;
}

void CostFunctionPyreneActuator::setCostGainTorqueConstraint(const commandMat_t& P_new)
{
    P = P_new;
}

void CostFunctionPyreneActuator::setTauLimit(const double& limit)
{
    tauLim = limit;
}

void CostFunctionPyreneActuator::setJointLimit(const double& limitUp, const double& limitDown)
{
    jointLim.push_back(limitUp);
    jointLim.push_back(limitDown);
}

void CostFunctionPyreneActuator::setJointVelLimit(const double& limitUp, const double& limitDown)
{
    jointVelLim.push_back(limitUp);
    jointVelLim.push_back(limitDown);
}

void CostFunctionPyreneActuator::computeTauConstraintsAndDeriv(const commandVec_t& U)
{
    double maxTau = 1 - alphaTau * (tauLim - (K*U[0] - offset_m));
    double minTau = 1 - alphaTau * ((K*U[0] - offset_m) + tauLim);   
    TauConstraints << exp(alphaTau * maxTau) + exp(alphaTau * minTau);
    dTauConstraints << alphaTau*alphaTau * K * (exp(alphaTau * maxTau) - exp(alphaTau * minTau));
    ddTauConstraints << pow(alphaTau, 4.0) * K*K * (exp(alphaTau * maxTau) + exp(alphaTau * minTau));
}

void CostFunctionPyreneActuator::computeConstraintsAndDeriv(const stateVec_t& X)
{
    double maxJoint = 1 - lambdaLimPos * (jointLim[0] - X[0]);
    double minJoint = 1 - lambdaLimPos * (X[0] - jointLim[1]);    
    Constraints[0] = exp(lambdaLimPos * maxJoint) + exp(lambdaLimPos * minJoint);

    double maxJointVel = 1 - lambdaLimVel * (jointVelLim[0] - X[0]);
    double minJointVel = 1 - lambdaLimVel * (X[0] - jointVelLim[1]);    
    Constraints[1] = exp(lambdaLimVel * maxJointVel) + exp(lambdaLimVel * minJointVel);

    double d0 = lambdaLimPos*lambdaLimPos * (exp(lambdaLimPos * maxJoint) - exp(lambdaLimPos * minJoint));
    double d1 = lambdaLimVel*lambdaLimVel * (exp(lambdaLimVel * maxJointVel) - exp(lambdaLimVel * minJointVel));
    dConstraints << d0, 0.0, 
                    0.0, d1; 

    double dd0 = pow(lambdaLimPos, 4.0) * Constraints[0];
    double dd1 = pow(lambdaLimVel, 4.0) * Constraints[1];
    ddConstraints << dd0, 0.0, 
                     0.0, dd1;               
}

void CostFunctionPyreneActuator::computeCostAndDeriv(const stateVec_t& X,const stateVec_t& Xdes, const commandVec_t& U)
{    
    computeConstraintsAndDeriv(X);
    computeTauConstraintsAndDeriv(U);
    running_cost =  ((X - Xdes).transpose() * Q * (X - Xdes) + U.transpose() * R * U + Constraints.transpose() * W * Constraints \
                    + TauConstraints.transpose() * P * TauConstraints)(0, 0);    

    lx = 2*Q*(X-Xdes) + (2.0*dConstraints.transpose()*W*Constraints);
    Eigen::Matrix<double, 2, 1> tempDD;
    tempDD = ddConstraints*W*Constraints; 
    Eigen::Matrix<double, 2, 2> dd;
    dd << tempDD[0], 0.0,
          0.0, tempDD[1];
    lxx = 2*Q + (2.0*(dConstraints.transpose()*W*dConstraints + dd));
    lu = 2*R*U + (2.0*dTauConstraints.transpose()*P*TauConstraints);
    luu = 2*R + (2.0*(ddTauConstraints.transpose()*P*TauConstraints + dTauConstraints.transpose()*P*dTauConstraints));
}

void CostFunctionPyreneActuator::computeFinalCostAndDeriv(const stateVec_t& X,const stateVec_t& Xdes)
{
    computeConstraintsAndDeriv(X);
    lx = Q*(X-Xdes) + (2.0*dConstraints.transpose()*W*Constraints);
    Eigen::Matrix<double, 2, 1> tempDD;
    tempDD = ddConstraints*W*Constraints; 
    Eigen::Matrix<double, 2, 2> dd;
    dd << tempDD[0], 0.0,
          0.0, tempDD[1];
    lxx = Q + (2.0*(dConstraints.transpose()*W*dConstraints + dd));
}
