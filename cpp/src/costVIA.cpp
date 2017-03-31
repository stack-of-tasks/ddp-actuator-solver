#include "costVIA.h"

CostVIA::CostVIA()
{
    Q <<    5.0,0.0,0.0,0.0,
            0.0,1e-4,0.0,0.0,
            0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0;

    R <<    1e-6,0.0,
            0.0,1e-9;

    lxx = Q;
    luu = R;
    lux << 0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0;
    lxu << 0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0;
    lx.setZero();
}

void CostVIA::computeAllCostDeriv(const stateVec_t& X,const stateVec_t& Xdes, const commandVec_t& U)
{
    lx = Q*(X-Xdes);
    lu = R*U;
}

void CostVIA::computeFinalCostDeriv(const stateVec_t& X,const stateVec_t& Xdes)
{
    lx = 1.0*Q*(X-Xdes);
    lxx = 1.0*Q;
}

