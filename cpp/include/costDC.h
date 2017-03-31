#ifndef COSTDC_H
#define COSTDC_H

#include "costfunction.h"

class CostDC : public CostFunction<double,3,1>
{
public:
    CostDC();
private:
    stateMat_t Q;
    commandMat_t R;
    double dt;
protected:
    // attributes //
public:
private:

protected:
    // methods //
public:
    void computeAllCostDeriv(const stateVec_t& X,const stateVec_t& Xdes, const commandVec_t& U);
    void computeFinalCostDeriv(const stateVec_t& X,const stateVec_t& Xdes);
private:
protected:
    // accessors //
public:

};

#endif // COSTROMEOPOS_H
