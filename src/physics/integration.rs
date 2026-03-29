//! Dormand–Prince **DOPRI5** — one fixed step of the 5th-order formula (6 RHS evaluations per step).
//!
//! This is the standard “RK45/ode45” family: embedded 4th order exists for error estimates; here we
//! apply only the **5th-order** update for a given step size `h` (suitable for real-time and headless).

/// Inertial accelerations in rotating ECEF: gravity + optional Coriolis + centrifugal.
#[inline]
pub fn inertial_accelerations(
    pos: glam::DVec3,
    vel: glam::DVec3,
    coriolis: bool,
    centrifugal: bool,
) -> glam::DVec3 {
    use super::earth::EARTH_OMEGA;
    use super::gravity::gravity_acceleration_ecef;

    let mut a = gravity_acceleration_ecef(pos);
    if coriolis {
        a += -2.0 * EARTH_OMEGA.cross(vel);
    }
    if centrifugal {
        a += -EARTH_OMEGA.cross(EARTH_OMEGA.cross(pos));
    }
    a
}

// ─── Dormand–Prince 5(4) coefficients (7-stage tableau; 5th-order update uses b₁…b₆, k₇ for FSAL only) ───

const DP_C2: f64 = 1.0 / 5.0;
const DP_C3: f64 = 3.0 / 10.0;
const DP_C4: f64 = 4.0 / 5.0;
const DP_C5: f64 = 8.0 / 9.0;

const DP_A21: f64 = 1.0 / 5.0;

const DP_A31: f64 = 3.0 / 40.0;
const DP_A32: f64 = 9.0 / 40.0;

const DP_A41: f64 = 44.0 / 45.0;
const DP_A42: f64 = -56.0 / 15.0;
const DP_A43: f64 = 32.0 / 9.0;

const DP_A51: f64 = 19_372.0 / 6_561.0;
const DP_A52: f64 = -25_360.0 / 2_187.0;
const DP_A53: f64 = 64_448.0 / 6_561.0;
const DP_A54: f64 = -212.0 / 729.0;

const DP_A61: f64 = 9_017.0 / 3_168.0;
const DP_A62: f64 = -355.0 / 33.0;
const DP_A63: f64 = 46_732.0 / 5_247.0;
const DP_A64: f64 = 49.0 / 176.0;
const DP_A65: f64 = -5_103.0 / 18_656.0;

// 5th-order weights (k₇ coefficient is 0 — not used in this fixed-step update)
const DP_B1: f64 = 35.0 / 384.0;
const DP_B2: f64 = 0.0;
const DP_B3: f64 = 500.0 / 1_113.0;
const DP_B4: f64 = 125.0 / 192.0;
const DP_B5: f64 = -2_187.0 / 6_784.0;
const DP_B6: f64 = 11.0 / 84.0;

#[inline]
fn combine_stages<const N: usize>(y: &[f64; N], k: &[[f64; N]; 6], a: &[f64; 6], h: f64) -> [f64; N] {
    let mut out = *y;
    for j in 0..N {
        for i in 0..6 {
            out[j] += h * a[i] * k[i][j];
        }
    }
    out
}

/// One DOPRI5 step for `y′ = f(t, y)` with **7** scalar components (e.g. position, velocity, mass).
pub fn dopri5_step_7(
    y: &mut [f64; 7],
    t: f32,
    h: f32,
    mut rhs: impl FnMut(f32, &[f64; 7]) -> [f64; 7],
) {
    let h = h as f64;
    let mut k = [[0f64; 7]; 6];

    k[0] = rhs(t, y);

    let y2 = combine_stages(y, &k, &[DP_A21, 0.0, 0.0, 0.0, 0.0, 0.0], h);
    k[1] = rhs(t + (h * DP_C2) as f32, &y2);

    let y3 = combine_stages(y, &k, &[DP_A31, DP_A32, 0.0, 0.0, 0.0, 0.0], h);
    k[2] = rhs(t + (h * DP_C3) as f32, &y3);

    let y4 = combine_stages(y, &k, &[DP_A41, DP_A42, DP_A43, 0.0, 0.0, 0.0], h);
    k[3] = rhs(t + (h * DP_C4) as f32, &y4);

    let y5 = combine_stages(y, &k, &[DP_A51, DP_A52, DP_A53, DP_A54, 0.0, 0.0], h);
    k[4] = rhs(t + (h * DP_C5) as f32, &y5);

    let y6 = combine_stages(y, &k, &[DP_A61, DP_A62, DP_A63, DP_A64, DP_A65, 0.0], h);
    k[5] = rhs(t + h as f32, &y6);

    let y_new = combine_stages(
        y,
        &k,
        &[DP_B1, DP_B2, DP_B3, DP_B4, DP_B5, DP_B6],
        h,
    );
    *y = y_new;
}

/// Same for **6** components (e.g. position + velocity, constant mass).
pub fn dopri5_step_6(
    y: &mut [f64; 6],
    t: f32,
    h: f32,
    mut rhs: impl FnMut(f32, &[f64; 6]) -> [f64; 6],
) {
    let h = h as f64;
    let mut k = [[0f64; 6]; 6];

    k[0] = rhs(t, y);

    let y2 = combine_stages(y, &k, &[DP_A21, 0.0, 0.0, 0.0, 0.0, 0.0], h);
    k[1] = rhs(t + (h * DP_C2) as f32, &y2);

    let y3 = combine_stages(y, &k, &[DP_A31, DP_A32, 0.0, 0.0, 0.0, 0.0], h);
    k[2] = rhs(t + (h * DP_C3) as f32, &y3);

    let y4 = combine_stages(y, &k, &[DP_A41, DP_A42, DP_A43, 0.0, 0.0, 0.0], h);
    k[3] = rhs(t + (h * DP_C4) as f32, &y4);

    let y5 = combine_stages(y, &k, &[DP_A51, DP_A52, DP_A53, DP_A54, 0.0, 0.0], h);
    k[4] = rhs(t + (h * DP_C5) as f32, &y5);

    let y6 = combine_stages(y, &k, &[DP_A61, DP_A62, DP_A63, DP_A64, DP_A65, 0.0], h);
    k[5] = rhs(t + h as f32, &y6);

    *y = combine_stages(
        y,
        &k,
        &[DP_B1, DP_B2, DP_B3, DP_B4, DP_B5, DP_B6],
        h,
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Scalar decay y' = -y embedded in 6D first component.
    #[test]
    fn dopri5_scalar_decay() {
        let h = 0.2f32;
        let mut y = [1.0f64, 0.0, 0.0, 0.0, 0.0, 0.0];
        dopri5_step_6(&mut y, 0.0, h, |_t, w| [-w[0], 0.0, 0.0, 0.0, 0.0, 0.0]);
        let analytical = (-h as f64).exp();
        assert!(
            (y[0] - analytical).abs() < 1e-7,
            "y[0]={} ref={}",
            y[0],
            analytical
        );
    }
}
