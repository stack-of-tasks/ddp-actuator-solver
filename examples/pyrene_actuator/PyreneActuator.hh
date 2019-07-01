#ifndef PYRENEACTUATOR_H
#define PYRENEACTUATOR_H

#include <ddp-actuator-solver/dynamicmodel.hh>

class PyreneActuator : public DynamicModel<double,2,1>
{
public:
    PyreneActuator();
    void setLoadParam(const double& mass, const double& coordX, const double& coordY);
    void setLoadMass(const double& mass);
    void removeLoad();
    stateVec_t computeStateDeriv(double&, const stateVec_t& X, const commandVec_t &U);
    stateVec_t computeNextState(double& dt, const stateVec_t& X, const commandVec_t &U);
    void computeModelDeriv(double& dt, const stateVec_t& X, const commandVec_t &U);
    stateMat_t computeTensorContxx(const stateVec_t& nextVx);
    commandMat_t computeTensorContuu(const stateVec_t& nextVx);
    commandR_stateC_t computeTensorContux(const stateVec_t& nextVx);

    static const double J;
    static const double K;
    static const double F_v;
    static const double F_s;
    static const double M;
    static const double c_x;
    static const double c_y;
    static const double mu;
    static const double g;

private:
    
    double L; //load
    double l_x;
    double l_y;
    stateVec_t Xreal;
    stateMat_t Id;
    stateMat_t A;
    stateMat_t Ad;
    stateR_commandC_t B;
    stateR_commandC_t Bd;
    double A13atan;
    double A33atan;
    stateMat_t fxBase;
    stateR_commandC_t fuBase;

    stateMat_t QxxCont;
    commandMat_t QuuCont;
    commandR_stateC_t QuxCont;

};

#endif // PYRENEACTUATOR_H
