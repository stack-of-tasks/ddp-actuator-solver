#ifndef COSTFUNCTIONPYRENEACTUATOR_H
#define COSTFUNCTIONPYRENEACTUATOR_H

#include <ddp-actuator-solver/costfunction.hh>
#include <vector>

class CostFunctionPyreneActuator : public CostFunction<double,2,1>
{
public:
    CostFunctionPyreneActuator();
    void computeCostAndDeriv(const stateVec_t& X,const stateVec_t& Xdes, const commandVec_t& U);
    void computeFinalCostAndDeriv(const stateVec_t& X,const stateVec_t& Xdes);
    void setTauLimit(double limit);
    void setJointLimit(double limitUp, double limitDown);
    void setJointVelLimit(double limitUp, double limitDown);
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
    std::vector<double> jointLim;
    std::vector<double> jointVelLim;
        
    double lambdaLim;

    stateVec_t Constraints;
    stateMat_t dConstraints;  
    stateMat_t ddConstraints;
    commandVec_t TauConstraints;
    commandVec_t dTauConstraints;  
    commandVec_t ddTauConstraints;

};

#endif // COSTFUNCTIONPYRENEACTUATOR_H
