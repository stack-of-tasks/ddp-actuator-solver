#ifndef MACCEPA_H
#define MACCEPA_H

#include "dynamicmodel.h"

class Maccepa : public DynamicModel<double,4,1>
{
public:
    Maccepa(double& mydt);
private:
protected:

    // attributes //
public:
private:
    double dt;
public:
    double k;
    //double B,C,P;
private:
    stateVec_t Xreal;
    stateMat_t Id;
    stateMat_t A;
    stateR_commandC_t B;
    double A13atan;
    double A33atan;
    stateMat_t fxBase;
    stateR_commandC_t fuBase;

    stateMat_t QxxCont;
    commandMat_t QuuCont;
    commandR_stateC_t QuxCont;

public: //temp
    stateMat_t Ad;
    stateR_commandC_t Bd;

protected:
    // methods //
public:
    stateVec_t computeNextState(double& dt, const stateVec_t& X, const commandVec_t &U);
    void computeAllModelDeriv(double& dt, const stateVec_t& X, const commandVec_t &U);
    stateMat_t computeTensorContxx(const stateVec_t& nextVx);
    commandMat_t computeTensorContuu(const stateVec_t& nextVx);
    commandR_stateC_t computeTensorContux(const stateVec_t& nextVx);
public:
    stateVec_t computeStateDeriv(double &dt, const stateVec_t& X, const commandVec_t &U);
protected:


};

#endif // MACCEPA_H
