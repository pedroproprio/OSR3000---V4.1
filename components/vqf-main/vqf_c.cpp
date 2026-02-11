#include "vqf_c.h"
#include "vqf.hpp"
#include <new>

struct vqf_handle_t
{
    VQF* obj;
};

extern "C" {

void vqf_params_init(vqf_params_t* params)
{
    if (!params) return;

    VQFParams cpp; // chama construtor original
    params->tauAcc = cpp.tauAcc;
    params->tauMag = cpp.tauMag;

    #ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    params->motionBiasEstEnabled = cpp.motionBiasEstEnabled;
    #else
    params->motionBiasEstEnabled = 0;
    #endif
    params->restBiasEstEnabled = cpp.restBiasEstEnabled;
    params->magDistRejectionEnabled = cpp.magDistRejectionEnabled;

    params->biasSigmaInit = cpp.biasSigmaInit;
    params->biasForgettingTime = cpp.biasForgettingTime;
    params->biasClip = cpp.biasClip;

    #ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    params->biasSigmaMotion = cpp.biasSigmaMotion;
    params->biasVerticalForgettingFactor = cpp.biasVerticalForgettingFactor;
    #else
    params->biasSigmaMotion = 0;
    params->biasVerticalForgettingFactor = 0;
    #endif

    params->biasSigmaRest = cpp.biasSigmaRest;

    params->restMinT = cpp.restMinT;
    params->restFilterTau = cpp.restFilterTau;
    params->restThGyr = cpp.restThGyr;
    params->restThAcc = cpp.restThAcc;

    params->magCurrentTau = cpp.magCurrentTau;
    params->magRefTau = cpp.magRefTau;
    params->magNormTh = cpp.magNormTh;
    params->magDipTh = cpp.magDipTh;

    params->magNewTime = cpp.magNewTime;
    params->magNewFirstTime = cpp.magNewFirstTime;
    params->magNewMinGyr = cpp.magNewMinGyr;

    params->magMinUndisturbedTime = cpp.magMinUndisturbedTime;
    params->magMaxRejectionTime = cpp.magMaxRejectionTime;
    params->magRejectionFactor = cpp.magRejectionFactor;
}

vqf_handle_t* vqf_init(float gyrTs, float accTs, float magTs)
{
    vqf_handle_t* h = new vqf_handle_t;

    accTs = accTs > 0 ? accTs : gyrTs;
    magTs = magTs > 0 ? magTs : gyrTs;

    if (!h)
        return nullptr;

    h->obj = new VQF(gyrTs, accTs, magTs);

    if (!h->obj)
    {
        delete h;
        return nullptr;
    }

    return h;
}

vqf_handle_t* vqf_init_custom(const vqf_params_t* params, float gyrTs, float accTs, float magTs)
{
    if (!params)
        return nullptr;

    VQFParams cpp;
    cpp.tauAcc = params->tauAcc;
    cpp.tauMag = params->tauMag;

    #ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    cpp.motionBiasEstEnabled = params->motionBiasEstEnabled;
    #endif

    cpp.restBiasEstEnabled = params->restBiasEstEnabled;
    cpp.magDistRejectionEnabled = params->magDistRejectionEnabled;

    cpp.biasSigmaInit = params->biasSigmaInit;
    cpp.biasForgettingTime = params->biasForgettingTime;
    cpp.biasClip = params->biasClip;

    #ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    cpp.biasSigmaMotion = params->biasSigmaMotion;
    cpp.biasVerticalForgettingFactor = params->biasVerticalForgettingFactor;
    #endif

    cpp.biasSigmaRest = params->biasSigmaRest;

    cpp.restMinT = params->restMinT;
    cpp.restFilterTau = params->restFilterTau;
    cpp.restThGyr = params->restThGyr;
    cpp.restThAcc = params->restThAcc;

    cpp.magCurrentTau = params->magCurrentTau;
    cpp.magRefTau = params->magRefTau;
    cpp.magNormTh = params->magNormTh;
    cpp.magDipTh = params->magDipTh;

    cpp.magNewTime = params->magNewTime;
    cpp.magNewFirstTime = params->magNewFirstTime;
    cpp.magNewMinGyr = params->magNewMinGyr;

    cpp.magMinUndisturbedTime = params->magMinUndisturbedTime;
    cpp.magMaxRejectionTime = params->magMaxRejectionTime;
    cpp.magRejectionFactor = params->magRejectionFactor;

    vqf_handle_t* h = new vqf_handle_t;
    h->obj = new VQF(cpp, gyrTs, accTs, magTs);
    return h;
}

void vqf_update(vqf_handle_t* handle, const float gyr[3], const float acc[3], const float mag[3])
{
    if (!handle || !handle->obj)
    return;

    handle->obj->update(gyr, acc, mag);
}

void vqf_get_quat9D(vqf_handle_t* handle, float out[4])
{
    if (!handle || !handle->obj || !out)
        return;

    handle->obj->getQuat9D(out);
}

void vqf_delete(vqf_handle_t* handle)
{
    if (!handle)
        return;

    if (handle->obj)
        delete handle->obj;

    delete handle;
}

}