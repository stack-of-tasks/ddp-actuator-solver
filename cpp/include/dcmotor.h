#ifndef DCMOTOR_H
#define DCMOTOR_H

#include "dynamicmodel.h"

class DCMotor : public DynamicModel<double,3,1>
{
public:
    DCMotor(double& mydt,bool noiseOnParameters=0);
private:
protected:

    // attributes //
public:
private:
    double dt;
private:
    double R;
    double L;
    double Jm;
    double fvm;
    double Kt;
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
    double getR() {return R;}
private:
    stateVec_t computeStateDeriv(double &dt, const stateVec_t& X, const commandVec_t &U);
protected:


};

#endif // ROMEOSIMPLEACTUATOR_H
