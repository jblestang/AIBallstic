//! Rocket thrust and mass flow (simplified solid-motor profile + pressure thrust).

use super::constants::{G0, P0};

/// ∫₀¹ w(τ) dτ for [`thrust_profile_weight`]; normalizes ṁ so total consumed propellant matches `fuel_mass`.
const THRUST_PROFILE_INTEGRAL: f64 = 0.0516 + 0.84 + 0.089;

#[inline]
fn thrust_profile_weight(tau: f64) -> f64 {
    let tau = tau.clamp(0.0, 1.0);
    if tau < 0.06 {
        lerp_prop(0.72, 1.0, tau / 0.06)
    } else if tau < 0.9 {
        1.0
    } else {
        lerp_prop(1.0, 0.78, (tau - 0.9) / 0.1)
    }
}

#[inline]
fn lerp_prop(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t.clamp(0.0, 1.0)
}

/// Instantaneous ṁ (kg/s) and reference ṁ_ref = m_fuel / t_burn for pressure-thrust scaling.
#[inline]
pub fn mass_flow_kg_s(fuel_mass: f64, burn_time: f32, t_in_stage: f32) -> (f64, f64) {
    let bt = burn_time as f64;
    if bt <= 0.0 {
        return (0.0, 0.0);
    }
    let tau = (t_in_stage as f64 / bt).clamp(0.0, 1.0);
    let w = thrust_profile_weight(tau);
    let mdot_ref = fuel_mass / bt;
    let mdot = mdot_ref * w / THRUST_PROFILE_INTEGRAL;
    (mdot, mdot_ref)
}

/// Thrust (N): vacuum momentum term minus ambient-pressure correction from Isp_vac / Isp_sl.
#[inline]
pub fn rocket_thrust_newton(
    mdot: f64,
    mdot_ref: f64,
    isp_vac: f64,
    isp_sl: f64,
    p_a: f64,
) -> f64 {
    let f_momentum_vac = mdot * isp_vac * G0;
    let delta_isp = isp_vac - isp_sl;
    let p_eff = p_a.clamp(0.0, P0);
    let pressure_loss = delta_isp * G0 * mdot_ref * (p_eff / P0);
    f_momentum_vac - pressure_loss
}
