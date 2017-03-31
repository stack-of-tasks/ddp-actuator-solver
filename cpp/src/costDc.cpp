#include "costDC.h"

CostDC::CostDC()
{
    Q <<    100.0,0.0,0.0,
            0.0,0.0,0.0,
            0.0,0.0,0.0;
    R << 0.1;

    lxx = Q;
    luu = R;
    lux << 0.0,0.0,0.0;
    lxu << 0.0,0.0,0.0;
    lx.setZero();
}

void CostDC::computeAllCostDeriv(const stateVec_t& X,const stateVec_t& Xdes, const commandVec_t& U)
{
    lx = Q*(X-Xdes);
    lu = R*U;
}

void CostDC::computeFinalCostDeriv(const stateVec_t& X,const stateVec_t& Xdes)
{
    lx = 1.0*Q*(X-Xdes);
    lxx = 1.0*Q;
}
