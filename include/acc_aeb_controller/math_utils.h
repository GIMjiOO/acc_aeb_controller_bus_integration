#pragma once
//==============================================================================
//  math_utils.h  —  Branch-light scalar helpers shared across the modules.
//  Header-only and ROS-free; everything here is constexpr/inline so the hot
//  path inlines to a handful of instructions with no call overhead.
//==============================================================================
#include <cmath>
#include "acc_aeb_controller/types.h"

namespace acc_aeb {

template <typename T>
[[nodiscard]] constexpr T clampVal(T v, T lo, T hi) noexcept {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Guarded division for the cold/variable-denominator cases (returns a fallback
// instead of inf/NaN). Hot-path divisions by a known-positive denominator are
// turned into reciprocal multiplies at the call site instead of using this.
[[nodiscard]] inline double safeDiv(double num, double den, double fallback = INF_D) noexcept {
    return (std::abs(den) > consts::SAFE_DIV_EPS) ? (num / den) : fallback;
}

[[nodiscard]] inline double computeTTC(double dist, double closing_spd) noexcept {
    return (closing_spd > consts::TTC_CLOSING_EPS) ? (dist / closing_spd) : INF_D;
}

// ── Kalman predict ────────────────────────────────────────────────────────────
// Propagates state x̂ and upper-triangular covariance P forward by dt seconds.
inline void kfPredict(KFState& s, double dt, double q_x, double q_v, double q_a) noexcept {
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;

    const double x_new = s.x + s.v * dt + 0.5 * s.a * dt2;
    const double v_new = s.v + s.a * dt;
    s.x = x_new;
    s.v = v_new;
    // a unchanged in constant-acceleration model

    const double Pxx = s.P[0], Pxv = s.P[1], Pxa = s.P[2];
    const double Pvv = s.P[3], Pva = s.P[4];
    const double Paa = s.P[5];

    const double FPxx = Pxx + Pxv*dt + Pxa*0.5*dt2;
    const double FPxv = Pxv + Pvv*dt + Pva*0.5*dt2;
    const double FPxa = Pxa + Pva*dt + Paa*0.5*dt2;

    s.P[0] = FPxx + FPxv*dt + FPxa*0.5*dt2  + q_x*dt5/20.0;
    s.P[1] = FPxv + FPxa*dt                  + q_x*dt4/8.0;
    s.P[2] = FPxa                             + q_x*dt3/6.0;
    s.P[3] = Pvv + 2.0*Pva*dt + Paa*dt2      + q_v*dt3/3.0;
    s.P[4] = Pva + Paa*dt                     + q_a*dt2/2.0;
    s.P[5] = Paa                              + q_a*dt;

    s.P[0] = std::max(s.P[0], 1e-6);
    s.P[3] = std::max(s.P[3], 1e-6);
    s.P[5] = std::max(s.P[5], 1e-6);
}

// ── Kalman update with one 2D measurement z = [x_meas, vx_meas] ──────────────
// H = [1 0 0; 0 1 0], R = diag(x_var, vx_var). Joseph form for stability.
inline void kfUpdate(KFState& s, double x_meas, double vx_meas,
                     double x_var, double vx_var) noexcept {
    const double inn_x  = x_meas  - s.x;
    const double inn_vx = vx_meas - s.v;

    const double S00 = s.P[0] + x_var;
    const double S01 = s.P[1];
    const double S11 = s.P[3] + vx_var;

    const double det = S00*S11 - S01*S01;
    if (std::abs(det) < 1e-10) return;
    const double inv_det = 1.0 / det;
    const double Si00 =  S11 * inv_det;
    const double Si01 = -S01 * inv_det;
    const double Si11 =  S00 * inv_det;

    // K = P·Hᵀ·S⁻¹  (P·Hᵀ = first two columns of P)
    const double PHt00 = s.P[0], PHt01 = s.P[1];
    const double PHt10 = s.P[1], PHt11 = s.P[3];
    const double PHt20 = s.P[2], PHt21 = s.P[4];

    const double K00 = PHt00*Si00 + PHt01*Si01;
    const double K01 = PHt00*Si01 + PHt01*Si11;
    const double K10 = PHt10*Si00 + PHt11*Si01;
    const double K11 = PHt10*Si01 + PHt11*Si11;
    const double K20 = PHt20*Si00 + PHt21*Si01;
    const double K21 = PHt20*Si01 + PHt21*Si11;

    s.x += K00*inn_x + K01*inn_vx;
    s.v += K10*inn_x + K11*inn_vx;
    s.a += K20*inn_x + K21*inn_vx;

    // Joseph form: P = (I-KH)·P⁻·(I-KH)ᵀ + K·R·Kᵀ
    const double IKH00 = 1.0-K00, IKH01 = -K01;
    const double IKH10 = -K10,    IKH11 = 1.0-K11;
    const double IKH20 = -K20,    IKH21 = -K21;

    const double p00 = s.P[0], p01 = s.P[1], p02 = s.P[2];
    const double p11 = s.P[3], p12 = s.P[4], p22 = s.P[5];

    auto dot3 = [](double a0, double a1, double a2,
                   double b0, double b1, double b2) noexcept {
        return a0*b0 + a1*b1 + a2*b2;
    };

    const double tmp00 = dot3(IKH00,IKH01,0, p00,p01,p02);
    const double tmp01 = dot3(IKH00,IKH01,0, p01,p11,p12);
    const double tmp02 = dot3(IKH00,IKH01,0, p02,p12,p22);
    const double tmp10 = dot3(IKH10,IKH11,0, p00,p01,p02);
    const double tmp11 = dot3(IKH10,IKH11,0, p01,p11,p12);
    const double tmp12 = dot3(IKH10,IKH11,0, p02,p12,p22);
    const double tmp20 = dot3(IKH20,IKH21,1, p00,p01,p02);
    const double tmp21 = dot3(IKH20,IKH21,1, p01,p11,p12);
    const double tmp22 = dot3(IKH20,IKH21,1, p02,p12,p22);

    s.P[0] = dot3(tmp00,tmp01,tmp02, IKH00,IKH01,0) + K00*K00*x_var + K01*K01*vx_var;
    s.P[1] = dot3(tmp00,tmp01,tmp02, IKH10,IKH11,0) + K00*K10*x_var + K01*K11*vx_var;
    s.P[2] = dot3(tmp00,tmp01,tmp02, IKH20,IKH21,1) + K00*K20*x_var + K01*K21*vx_var;
    s.P[3] = dot3(tmp10,tmp11,tmp12, IKH10,IKH11,0) + K10*K10*x_var + K11*K11*vx_var;
    s.P[4] = dot3(tmp10,tmp11,tmp12, IKH20,IKH21,1) + K10*K20*x_var + K11*K21*vx_var;
    s.P[5] = dot3(tmp20,tmp21,tmp22, IKH20,IKH21,1) + K20*K20*x_var + K21*K21*vx_var;

    s.a = clampVal(s.a, -consts::KF_A_REL_CLAMP_MPS2, consts::KF_A_REL_CLAMP_MPS2);
    s.P[0] = std::max(s.P[0], 1e-6);
    s.P[3] = std::max(s.P[3], 1e-6);
    s.P[5] = std::max(s.P[5], 1e-6);
}

// ── Initialise KFState from a first detection ─────────────────────────────────
inline void kfInit(KFState& s, double x, double v, double x_var, double vx_var) noexcept {
    s.x = x;
    s.v = v;
    s.a = 0.0;
    s.P[0] = x_var  * 4.0;
    s.P[1] = 0.0;
    s.P[2] = 0.0;
    s.P[3] = vx_var * 4.0;
    s.P[4] = 0.0;
    s.P[5] = 2.0;
    s.initialised = true;
}

} // namespace acc_aeb
