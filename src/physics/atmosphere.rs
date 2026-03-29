//! Air density, pressure, and speed of sound from ISA / USSA + thermosphere extension.

use super::constants::{G0, GAMMA, P0, R_AIR};

/// US Standard Atmosphere 1976, geometric altitude 0–86 km: T (K) and P (Pa).
fn ussa1976_temperature_pressure(z: f64) -> (f64, f64) {
    const G: f64 = G0;
    const R: f64 = R_AIR;
    let z = z.clamp(0.0, 86_000.0);

    let p11 = P0 * (216.65_f64 / 288.15_f64).powf(-G / (R * -0.0065));
    let p20 = p11 * f64::exp(-G * 9_000.0 / (R * 216.65));
    let p32 = p20 * (228.65_f64 / 216.65_f64).powf(-G / (R * 0.001));
    let p47 = p32 * (270.65_f64 / 228.65_f64).powf(-G / (R * 0.0028));
    let p51 = p47 * f64::exp(-G * 4_000.0 / (R * 270.65));
    let p71 = p51 * (214.65_f64 / 270.65_f64).powf(-G / (R * -0.0028));

    if z < 11_000.0 {
        let t = 288.15 - 0.0065 * z;
        let p = P0 * (t / 288.15_f64).powf(-G / (R * -0.0065));
        (t, p)
    } else if z < 20_000.0 {
        let t = 216.65;
        let p = p11 * f64::exp(-G * (z - 11_000.0) / (R * t));
        (t, p)
    } else if z < 32_000.0 {
        let t = 216.65 + 0.001 * (z - 20_000.0);
        let p = p20 * (t / 216.65_f64).powf(-G / (R * 0.001));
        (t, p)
    } else if z < 47_000.0 {
        let t = 228.65 + 0.0028 * (z - 32_000.0);
        let p = p32 * (t / 228.65_f64).powf(-G / (R * 0.0028));
        (t, p)
    } else if z < 51_000.0 {
        let t = 270.65;
        let p = p47 * f64::exp(-G * (z - 47_000.0) / (R * t));
        (t, p)
    } else if z < 71_000.0 {
        let t = 270.65 - 0.0028 * (z - 51_000.0);
        let p = p51 * (t / 270.65_f64).powf(-G / (R * -0.0028));
        (t, p)
    } else {
        let t = 214.65 - 0.002 * (z - 71_000.0);
        let p = p71 * (t / 214.65_f64).powf(-G / (R * -0.002));
        (t, p)
    }
}

fn thermosphere_properties(z: f64, t86: f64, rho86: f64) -> (f64, f64, f64) {
    let z = z.min(5_000_000.0);
    let z_km = z / 1000.0;

    const Z_KM: &[f64] = &[100.0, 120.0, 150.0, 200.0, 300.0, 400.0, 500.0, 600.0, 800.0, 1000.0, 1500.0, 2000.0];
    const RHO: &[f64] = &[
        5.60e-7,
        2.99e-8,
        2.08e-9,
        3.58e-10,
        1.84e-11,
        3.92e-12,
        1.03e-12,
        3.31e-13,
        1.17e-14,
        3.56e-15,
        2.0e-16,
        2.5e-17,
    ];
    const T_K: &[f64] = &[195.0, 360.0, 634.0, 854.0, 976.0, 996.0, 999.0, 982.0, 964.0, 950.0, 850.0, 800.0];

    debug_assert_eq!(Z_KM.len(), RHO.len());
    debug_assert_eq!(Z_KM.len(), T_K.len());

    let lr86 = rho86.max(1e-30).ln();
    let lr100 = RHO[0].ln();

    let (rho, t) = if z_km <= Z_KM[0] {
        let u = (z_km - 86.0) / (Z_KM[0] - 86.0);
        let tt = lerp(t86, T_K[0], u);
        let lr = lerp(lr86, lr100, u);
        (lr.exp(), tt)
    } else if z_km >= Z_KM[Z_KM.len() - 1] {
        let lr = RHO[RHO.len() - 1].ln();
        let tt = T_K[T_K.len() - 1];
        let scale_km = 250.0;
        let extra = (z_km - Z_KM[Z_KM.len() - 1]) / scale_km;
        (lr.exp() * f64::exp(-extra), tt)
    } else {
        let mut j = 0usize;
        while j + 1 < Z_KM.len() && z_km > Z_KM[j + 1] {
            j += 1;
        }
        let z0 = Z_KM[j];
        let z1 = Z_KM[j + 1];
        let u = ((z_km - z0) / (z1 - z0)).clamp(0.0, 1.0);
        let lr = RHO[j].ln() * (1.0 - u) + RHO[j + 1].ln() * u;
        let tt = T_K[j] * (1.0 - u) + T_K[j + 1] * u;
        (lr.exp(), tt)
    };

    let p = rho * R_AIR * t;
    let sound = (GAMMA * R_AIR * t).sqrt();
    (rho, sound, p)
}

#[inline]
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t.clamp(0.0, 1.0)
}

/// Density (kg/m³), speed of sound (m/s), pressure (Pa) at **ellipsoidal** altitude (m).
pub fn get_isa_properties(altitude: f64) -> (f64, f64, f64) {
    let h = altitude.max(0.0);
    const Z_TOP: f64 = 86_000.0;

    let (t86, p86) = ussa1976_temperature_pressure(Z_TOP);
    let rho86 = p86 / (R_AIR * t86);

    if h <= Z_TOP {
        let (t, p) = ussa1976_temperature_pressure(h);
        let rho = p / (R_AIR * t);
        let sound_speed = (GAMMA * R_AIR * t).sqrt();
        return (rho, sound_speed, p);
    }

    thermosphere_properties(h, t86, rho86)
}

#[cfg(test)]
mod tests {
    use super::super::constants::{P0, R_AIR, GAMMA, T0};
    use super::*;

    #[test]
    fn sea_level_isa() {
        let (rho, a, p) = get_isa_properties(0.0);
        assert!((rho - 1.225).abs() < 0.02);
        assert!((p - P0).abs() < 1.0);
        assert!((a - (GAMMA * R_AIR * T0).sqrt()).abs() < 0.5);
    }

    #[test]
    fn ussa_32km_order_of_magnitude() {
        let (_rho, _a, p) = get_isa_properties(32_000.0);
        assert!(p > 400.0 && p < 1_400.0);
    }

    #[test]
    fn thermosphere_continuity_at_86km() {
        let (r0, a0, p0) = get_isa_properties(86_000.0);
        let (r1, a1, p1) = get_isa_properties(86_000.1);
        let dr = (r1 - r0).abs() / r0;
        assert!(dr < 0.02, "density jump at 86 km: {dr}");
        let da = (a1 - a0).abs() / a0;
        assert!(da < 0.02, "sound speed jump at 86 km: {da}");
        let dp = (p1 - p0).abs() / p0;
        assert!(dp < 0.02, "pressure jump at 86 km: {dp}");
    }

    #[test]
    fn thermosphere_thin_at_400km() {
        let (rho, _, _) = get_isa_properties(400_000.0);
        assert!(rho < 1e-11 && rho > 1e-13);
    }
}
