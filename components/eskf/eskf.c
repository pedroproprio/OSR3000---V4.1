#include "eskf.h"

#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define Q_N 6
static float Q[Q_N]; // only nonzero and unique entries
static float k0=0.0f, k1=0.0f, gdt=0.0f; // precomputed constants for prediction step

static float ba_constraint = 0.0f; // Limit for accelerometer bias

// Symmetrize covariance matrix to prevent numerical issues
static void symmetrize(float P[KF_N][KF_N]) {
    for (int i = 0; i < KF_N; i++)
        for (int j = i + 1; j < KF_N; j++)
        {
            float val = (P[i][j] + P[j][i]) * 0.5f;
            P[i][j] = val;
            P[j][i] = val;
        }
}

static inline bool struct_floats_positive(const eskf_var_t *v)
{
    const float *p = (const float *)v;
    for (size_t i = 0; i < sizeof(*v)/sizeof(float); i++)
        if (p[i] <= 0.0f)
            return false;
    return true;
}

esp_err_t eskf_init(eskf_t *eskf)
{
    if(eskf->initialized)
        return ESP_OK; // already initialized, do nothing

    // Validate config parameters
    ESP_ARG_CHECK(eskf);
    ESP_ARG_CHECK(struct_floats_positive(&eskf->cfg.var));
    if (eskf->cfg.dt <= 0.0f || eskf->cfg.g <= 0.0f) return ESP_ERR_INVALID_ARG;
        
    eskf_config_t cfg = eskf->cfg;
    *eskf = (eskf_t){0};
    eskf->cfg = cfg;

    float (*P)[KF_N] = eskf->P;
    float g = eskf->cfg.g;

    // Initial covariance
    P[0][0] = cfg.var.bar * 10.0f;    // altitude
    P[1][1] = cfg.var.gps_vz * 10.0f; // velocity
    P[2][2] = cfg.var.ba * 10.0f;     // accelerometer bias
    P[3][3] = cfg.var.bb * 10.0f;     // barometer bias
    P[4][4] = cfg.var.θe * 10.0f;     // angular error

    float dt = eskf->cfg.dt;
    k0 = dt * dt * 0.5f; // t²/2
    k1 = g * k0; // gt²/2
    gdt = g * dt;

    // Q matrix
    Q[0] = eskf->cfg.var.acc * dt * dt * dt * dt * 0.25f; // Q00
    Q[1] = eskf->cfg.var.acc * dt * dt * dt * 0.5f;       // Q01 = Q10
    Q[2] = eskf->cfg.var.acc * dt * dt;                   // Q11
    Q[3] = eskf->cfg.var.ba * dt;                         // Q22
    Q[4] = eskf->cfg.var.bb * dt;                         // Q33
    Q[5] = eskf->cfg.var.θe * dt;                         // Q44

    ba_constraint = sqrtf(eskf->cfg.var.acc) * 3.0f; // 3σ
    eskf->initialized = true;

    return ESP_OK;
}

void eskf_predict(eskf_t *eskf, float az, float xQ)
{
    if (!eskf->initialized)
        return;

    float dt = eskf->cfg.dt;
    float g = eskf->cfg.g;

    bool idle = false;
    static uint16_t idle_counter = 0; // Counter for idle state
    if (fabsf(az) < ba_constraint) // consider idle if acceleration is very low
    {
        if (idle_counter < eskf->cfg.idle_samples)
            idle_counter++;
        else
        {
            idle = true;
            eskf_update_gps(eskf, eskf->x.h, 0.0f, 0.001f); // ZUPT update to correct drift when idle
            eskf->P[4][4] = 0.0001f;
            eskf->δx.θe = 0.0f;
            eskf->x.θe = 0.0f; // assumes no angular error when idle
        }
    }
    else if (idle_counter > 0)
        idle_counter = 0; // reset counter if not idle

    eskf_state_t *x = &eskf->x;
    eskf_error_t *δx = &eskf->δx;
    float (*P)[KF_N] = eskf->P;
    
    // State prediction
    //float dh = δx->h;
    float dv = δx->vz;
    float ba = δx->ba;
    float dth = δx->θe;

    float a = idle ? 0.0f : az - ba - dth * g; // 0 when idle, else a = az - ba - gθe
    x->h += x->vz * dt + a * k0; // h += vt + 0.5at²
    x->vz += a * dt; // v += at

    δx->h += dt * dv; // δh += dt*δv
    δx->vz -= dt * ba - gdt * dth; // δv += -dt*δba - gdt*δθe
    
    // Update apogee
    static float previous_vz = 0.0f;
    if (previous_vz > 0 && x->vz <= 0) // detect apogee at the transition from positive to non-positive vertical velocity
        if (x->h > x->apogee) // update apogee if current altitude is higher
            x->apogee = x->h;
    previous_vz = x->vz;

    // Scale process noise
    if (xQ < 1.0f) xQ = 1.0f;
    float Qs[Q_N];
    for (int i = 0; i < Q_N; i++)
        Qs[i] = Q[i] * xQ;

    // #1: F*P
    float FP[KF_N][KF_N];
    for(int j=0;j<KF_N;j++)
    {
        FP[0][j] = P[0][j] + dt * P[1][j] - k0 * P[2][j] - k1 * P[4][j];
        FP[1][j] = P[1][j] - dt * P[2][j] - gdt * P[4][j];
        FP[2][j] = P[2][j];
        FP[3][j] = P[3][j];
        FP[4][j] = P[4][j];
    }

    // #2: F*P*Fᵀ
    for(int i=0;i<KF_N;i++)
    {
        P[i][0] = FP[i][0] + dt * FP[i][1] - k0 * FP[i][2] - k1 * FP[i][4];
        P[i][1] = FP[i][1] - dt * FP[i][2] - gdt * FP[i][4];
        P[i][2] = FP[i][2];
        P[i][3] = FP[i][3];
        P[i][4] = FP[i][4];
    }

    // #3: F*P*Fᵀ + Q
    P[0][0] += Qs[0];
    P[0][1] += Qs[1];
    P[1][0] += Qs[1];
    P[1][1] += Qs[2];
    P[2][2] += Qs[3];
    P[3][3] += Qs[4];
    P[4][4] += Qs[5];
    
    symmetrize(P);
}

// Mutual code for barometer and GPS updates
static void update(eskf_t *eskf)
{
    // x += δx
    eskf_state_t *x = &eskf->x;
    eskf_error_t *δx = &eskf->δx;

    // Constraint angular error to prevent divergence
    if (δx->θe > 0.24f) δx->θe = 0.24f; // ≈ 14°
    else if (δx->θe < -0.24f) δx->θe = -0.24f;
    
    // Update nominal state with error state
    x->h += δx->h;
    x->vz += δx->vz;
    x->ba += δx->ba;
    x->bb = δx->bb;
    x->θe += δx->θe;

    // Constrain accelerometer bias to prevent divergence
    if (x->ba > ba_constraint)  x->ba = ba_constraint;
    else if (x->ba < -ba_constraint) x->ba = -ba_constraint;

    // Reset errors
    δx->h = 0;
    δx->vz = 0;
    δx->ba = 0;
    δx->θe = 0;
    // Note: barometer bias is not reset to 0, it is estimated as part of the state
    // That is because it is not used in the model integration, only in the measurement update,
    // so it does not cause instability if not reset
}

bool eskf_update_bar(eskf_t *eskf, float h, float xR)
{
    if (!eskf->initialized)
        return false;

    eskf_error_t *δx = &eskf->δx;
    float (*P)[KF_N] = eskf->P;

    if (xR < 1.0f) xR = 1.0f;
    float R = eskf->cfg.var.bar * xR;

    // Residue
    float r = h - eskf->x.h; // r = h_bar - h_nominal

    // Innovation
    float y = r - δx->h - δx->bb;

    // H = [1 0 0 1 0]
    // S = HPHᵀ + R
    float S = P[0][0] + P[0][3] + P[3][0] + P[3][3] + R;

    // Innovation gate 1D
    // yᵀS⁻¹y
    float igt = eskf->cfg.igt;
    if (igt != 0.0f)
        if (y * y / S > igt * igt)
            return false;

    // K = PHᵀS⁻¹
    float K[KF_N];
    float invS = 1.0f / S;
    K[0] = (P[0][0] + P[0][3]) * invS;
    K[1] = (P[1][0] + P[1][3]) * invS;
    K[2] = (P[2][0] + P[2][3]) * invS;
    K[3] = (P[3][0] + P[3][3]) * invS;
    K[4] = (P[4][0] + P[4][3]) * invS;

    // δx += K*y
    δx->h += K[0]*y;
    δx->vz += K[1]*y;
    δx->ba += K[2]*y;
    δx->bb += K[3]*y;
    δx->θe += K[4]*y;

    update(eskf);

    // (I-KH)P(I-KH)ᵀ + KRKᵀ
    float HP[KF_N];
    for (int j = 0; j < KF_N; j++)
        HP[j] = P[0][j] + P[3][j];

    for (int i = 0; i < KF_N; i++)
        for (int j = 0; j < KF_N; j++)
            P[i][j] +=
                - K[i] * HP[j]
                - HP[i] * K[j]
                + K[i] * (HP[0] + HP[3] + R) * K[j];

    symmetrize(P);
    return true;
}

bool eskf_update_gps(eskf_t *eskf, float h, float vz, float xR)
{
    if (!eskf->initialized)
        return false;
    
    eskf_state_t *x = &eskf->x;
    eskf_error_t *δx = &eskf->δx;
    float (*P)[KF_N] = eskf->P;

    float R[2];
    if (xR < 1.0f) xR = 1.0f;
    R[0] = eskf->cfg.var.gps_h * xR;
    R[1] = eskf->cfg.var.gps_vz * xR;

    // Residues
    float rh = h - x->h; // rh = h_gps - h_nominal
    float rv = vz - x->vz; // rv = v_gps - v_nominal

    // Innovation
    float y0 = rh - δx->h;
    float y1 = rv - δx->vz;

    // H = [1 0 0 0 0
    //      0 1 0 0 0]
    // S = HPHᵀ + R
    float S00 = P[0][0] + R[0];
    float S01 = P[0][1];
    float S10 = P[1][0];
    float S11 = P[1][1] + R[1];

    // Inverse 2x2
    float det = S00*S11 - S01*S10;
    float invDet = 1.0f / det;

    // Innovation gate 2D
    // yᵀS⁻¹y
    float igt = eskf->cfg.igt;
    if (igt != 0.0f)
    {
        float maha = (y0*(y0*S11 - y1*S10) + y1*(y1*S00 - y0*S01)) * invDet;
        if (maha > igt * igt)
            return false;
    }

    // K = PHᵀS⁻¹
    float K[KF_N][2];
    for(int i=0; i<KF_N; i++) {
        K[i][0] = (P[i][0]*S11 - P[i][1]*S10) * invDet;
        K[i][1] = (P[i][1]*S00 - P[i][0]*S01) * invDet;
    }

    // δx += K*y
    δx->h += K[0][0]*y0 + K[0][1]*y1;
    δx->vz += K[1][0]*y0 + K[1][1]*y1;
    δx->ba += K[2][0]*y0 + K[2][1]*y1;
    δx->bb += K[3][0]*y0 + K[3][1]*y1;
    δx->θe += K[4][0]*y0 + K[4][1]*y1;

    update(eskf);

    // (I-KH)P(I-KH)ᵀ + KRKᵀ
    float P0[KF_N], P1[KF_N];
    for (int i=0;i<KF_N;i++)
    {
        P0[i] = P[i][0];
        P1[i] = P[i][1];
    }

    for (int i = 0; i < KF_N; i++) 
        for (int j = 0; j < KF_N; j++) {
            P[i][j] +=
                - K[i][0] * P0[j]
                - K[i][1] * P1[j]
                - P0[i] * K[j][0]
                - P1[i] * K[j][1]
                + K[i][0] * (S00 * K[j][0] + S01 * K[j][1])
                + K[i][1] * (S10 * K[j][0] + S11 * K[j][1]);
        }

    symmetrize(P);
    return true;
}