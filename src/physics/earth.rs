//! WGS84 geodetic ↔ ECEF and Earth rotation for the **game ECEF** frame.

use glam::DVec3;

use super::constants::{WGS84_A, WGS84_B, WGS84_E2, WGS84_EP2};

/// Earth's angular velocity (rad/s): rotation about **+Y** (North Pole in game ECEF).
pub const EARTH_OMEGA: DVec3 = DVec3::new(0.0, 7.292_115_9e-5, 0.0);

/// Game ECEF: **+Y = North Pole**, prime meridian in +X / −Z (lon 0° → +X).
/// `lat`, `lon` in degrees; `h` = ellipsoidal height (m) above WGS84.
#[inline]
pub fn geodetic_to_ecef(lat: f64, lon: f64, h: f64) -> DVec3 {
    let phi = lat.to_radians();
    let lam = lon.to_radians();
    let sin_p = phi.sin();
    let cos_p = phi.cos();
    let sin_l = lam.sin();
    let cos_l = lam.cos();

    let n = WGS84_A / (1.0 - WGS84_E2 * sin_p * sin_p).sqrt();
    let x = (n + h) * cos_p * cos_l;
    let y = (n + h) * cos_p * sin_l;
    let z = (n * (1.0 - WGS84_E2) + h) * sin_p;
    DVec3::new(x, z, -y)
}

/// Bowring-style ECEF → geodetic (radians). Input is game ECEF.
pub fn ecef_to_geodetic_rad_h(p: DVec3) -> (f64, f64, f64) {
    let x = p.x;
    let y = p.y;
    let z = p.z;
    let y_wgs = -z;
    let z_wgs = y;
    let p_xy = (x * x + y_wgs * y_wgs).sqrt();

    if p_xy < 1e-12 {
        let lat = std::f64::consts::FRAC_PI_2.copysign(z_wgs);
        let lon = 0.0;
        let h = z_wgs.abs() - WGS84_B;
        return (lat, lon, h);
    }

    let lon = y_wgs.atan2(x);
    let theta = (WGS84_A * z_wgs).atan2(WGS84_B * p_xy);
    let st = theta.sin();
    let ct = theta.cos();
    let lat = (z_wgs + WGS84_EP2 * WGS84_B * st * st * st)
        .atan2(p_xy - WGS84_E2 * WGS84_A * ct * ct * ct);
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    let h = p_xy / cos_lat - n;
    (lat, lon, h)
}

/// Ellipsoidal height (m) above WGS84; negative inside the ellipsoid.
#[inline]
pub fn ellipsoid_height_m(p: DVec3) -> f64 {
    ecef_to_geodetic_rad_h(p).2
}

#[inline]
pub fn ecef_to_geodetic_deg_h(p: DVec3) -> (f64, f64, f64) {
    let (lat, lon, h) = ecef_to_geodetic_rad_h(p);
    (lat.to_degrees(), lon.to_degrees(), h)
}

/// Geodetic latitude and longitude (degrees) from game ECEF.
#[inline]
pub fn ecef_to_geodetic_deg(p: DVec3) -> (f64, f64) {
    let (lat, lon, _) = ecef_to_geodetic_deg_h(p);
    (lat, lon)
}
