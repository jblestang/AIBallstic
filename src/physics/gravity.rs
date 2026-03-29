//! Central gravity plus zonal harmonics J₂–J₄ in rotating-frame ECEF.

use glam::DVec3;

use super::constants::{EARTH_J2, EARTH_J3, EARTH_J4, GRAVITY_CONSTANT, WGS84_A};

#[inline]
fn game_ecef_to_conv_z_north(r: DVec3) -> (f64, f64, f64) {
    (r.x, -r.z, r.y)
}

#[inline]
fn conv_accel_z_north_to_game(ax: f64, ay_eq: f64, az_n: f64) -> DVec3 {
    DVec3::new(ax, az_n, -ay_eq)
}

/// Gravitational acceleration (m/s²): central term + J₂ + J₃ + J₄ (EGM96/WGS84-like zonal terms).
/// Frame: game ECEF (+Y north); spherical-harmonic formulas use conventional Z-north, then map back.
#[inline]
pub fn gravity_acceleration_ecef(r: DVec3) -> DVec3 {
    let r_mag = r.length();
    if r_mag < 1.0 {
        return DVec3::ZERO;
    }
    let a_central = -r * (GRAVITY_CONSTANT / (r_mag * r_mag * r_mag));

    let (x, y_eq, z_n) = game_ecef_to_conv_z_north(r);
    let inv_r = 1.0 / r_mag;
    let z_over_r = z_n * inv_r;
    let zor2 = z_over_r * z_over_r;

    let fac_j2 = -1.5 * EARTH_J2 * GRAVITY_CONSTANT * WGS84_A * WGS84_A / r_mag.powi(5);
    let term_xy = 1.0 - 5.0 * zor2;
    let term_z = 3.0 - 5.0 * zor2;
    let ax_j2 = fac_j2 * x * term_xy;
    let ay_j2 = fac_j2 * y_eq * term_xy;
    let az_j2 = fac_j2 * z_n * term_z;
    let a_j2 = conv_accel_z_north_to_game(ax_j2, ay_j2, az_j2);

    let cos_phi = z_over_r;
    let j3_brack = 7.0 * cos_phi.powi(3) - 3.0 * cos_phi;
    let fac_j3 = 0.5 * EARTH_J3 * GRAVITY_CONSTANT * WGS84_A.powi(3) / r_mag.powi(5);
    let ax_j3 = fac_j3 * 5.0 * x * inv_r * j3_brack;
    let ay_j3 = fac_j3 * 5.0 * y_eq * inv_r * j3_brack;
    let az_j3 = fac_j3 * 3.0 * ((35.0 / 3.0) * cos_phi.powi(4) - 10.0 * cos_phi.powi(2) + 1.0);
    let a_j3 = conv_accel_z_north_to_game(ax_j3, ay_j3, az_j3);

    let r2 = r_mag * r_mag;
    let r4 = r2 * r2;
    let z2 = z_n * z_n;
    let n_j4 = 35.0 * z_n.powi(4) - 30.0 * z2 * r2 + 3.0 * r4;
    let dndx = -60.0 * x * z2 + 12.0 * x * r2;
    let dndy = -60.0 * y_eq * z2 + 12.0 * y_eq * r2;
    let dndz = 80.0 * z_n.powi(3) - 48.0 * z_n * r2;
    let c_j4 = GRAVITY_CONSTANT * EARTH_J4 * WGS84_A.powi(4) / 8.0;
    let r11 = r_mag.powi(11);
    let gx_j4 = c_j4 * (dndx * r2 - 9.0 * x * n_j4) / r11;
    let gy_j4 = c_j4 * (dndy * r2 - 9.0 * y_eq * n_j4) / r11;
    let gz_j4 = c_j4 * (dndz * r2 - 9.0 * z_n * n_j4) / r11;
    let a_j4 = conv_accel_z_north_to_game(-gx_j4, -gy_j4, -gz_j4);

    a_central + a_j2 + a_j3 + a_j4
}
