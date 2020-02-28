#ifndef PYRENEACTUATOR_H
#define PYRENEACTUATOR_H

#include <ddp-actuator-solver/dynamicmodel.hh>

class pyreneActuator : public DynamicModel<double,2,1>
{
public:
    pyreneActuator();
    void setLoadParam(const double& mass, const double& coordX, const double& coordY);
    void setLoadMass(const double& mass);
    void removeLoad();
    stateVec_t computeStateDeriv(double&, const stateVec_t& X, const commandVec_t &U);
    stateVec_t computeNextState(double& dt, const stateVec_t& X, const commandVec_t &U);
    void computeModelDeriv(double& dt, const stateVec_t& X, const commandVec_t &U);
    stateMat_t computeTensorContxx(const stateVec_t& nextVx);
    commandMat_t computeTensorContuu(const stateVec_t& nextVx);
    commandR_stateC_t computeTensorContux(const stateVec_t& nextVx);

    static const double J_j;
    static const double K;
    static const double F_vj;
    static const double F_sj;
    static const double J_m;
    static const double F_vm;
    static const double F_sm;
    static const double offset_m;
    static const double offset_j;
    static const double M;
    static const double c_x;
    static const double c_y;
    static const double mu;
    static const double g;

private:
    double J;
    double F_v;
    double F_s;
    double L; //load
    double l_x;
    double l_y;

    stateMat_t QxxCont;
    commandMat_t QuuCont;
    commandR_stateC_t QuxCont;

};

#endif // PYRENEACTUATOR_H
