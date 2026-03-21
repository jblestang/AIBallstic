//! Headless trajectory integration matching `physics_system` in `main.rs` (fixed timestep).

use glam::DVec3;

use crate::missiles::{
    BallisticMissilePhysics, EARTH_OMEGA, EARTH_RADIUS, FlightPhase, GRAVITY_CONSTANT, MissileSpecs,
    PhysicsModel,
};

/// Settings for offline runs (game UI uses variable frame `dt` × `time_scale`; this is fixed-`dt`).
#[derive(Debug, Clone)]
pub struct SimParams {
    /// Simulated seconds per integration step.
    pub dt: f64,
    pub coriolis_enabled: bool,
    pub centrifugal_enabled: bool,
    pub max_time_s: f32,
}

impl Default for SimParams {
    fn default() -> Self {
        Self {
            dt: 0.02,
            coriolis_enabled: true,
            centrifugal_enabled: true,
            max_time_s: 20_000.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct GroundImpactResult {
    pub landed: bool,
    pub flight_time_s: f32,
    /// Last integrated position (may be slightly inside the sphere when `landed`).
    pub position_ecef: DVec3,
    pub impact_lat_deg: f64,
    pub impact_lon_deg: f64,
    /// Great-circle miss distance to the aim point used in `BallisticMissilePhysics`.
    pub miss_distance_m: f64,
}

/// Inverse of `geodetic_to_ecef` for the game's ECEF convention.
pub fn ecef_to_geodetic_deg(p: DVec3) -> (f64, f64) {
    let r = p.length();
    if r < 1.0 {
        return (0.0, 0.0);
    }
    let lat = (p.y / r).clamp(-1.0, 1.0).asin().to_degrees();
    let lon = p.z.atan2(-p.x).to_degrees();
    (lat, lon)
}

fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let r = EARTH_RADIUS;
    let p1 = lat1.to_radians();
    let p2 = lat2.to_radians();
    let dp = (lat2 - lat1).to_radians();
    let dl = (lon2 - lon1).to_radians();
    let a = (dp / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();
    r * c
}

/// Integrate until the missile intersects the spherical Earth (`r < EARTH_RADIUS` after 1 s), or `max_time_s`.
pub fn simulate_until_ground_impact(
    specs: MissileSpecs,
    launch_ecef: DVec3,
    target_ecef: DVec3,
    params: &SimParams,
) -> GroundImpactResult {
    let model = BallisticMissilePhysics::new(specs, target_ecef);
    let (tgt_lat, tgt_lon) = ecef_to_geodetic_deg(target_ecef);
    let dt = params.dt as f32;

    let mut pos = launch_ecef;
    let mut vel = DVec3::ZERO;
    let mut mass = model.specs().total_mass();
    let mut timer = 0.0f32;
    let mut _phase = FlightPhase::Boost;

    while timer < params.max_time_s {
        let r = pos.length();
        if r < EARTH_RADIUS && timer > 1.0 {
            let (lat, lon) = ecef_to_geodetic_deg(pos);
            let miss = haversine_m(lat, lon, tgt_lat, tgt_lon);
            return GroundImpactResult {
                landed: true,
                flight_time_s: timer,
                position_ecef: pos,
                impact_lat_deg: lat,
                impact_lon_deg: lon,
                miss_distance_m: miss,
            };
        }

        let gravity_dir = -pos.normalize();
        let gravity_accel = gravity_dir * (GRAVITY_CONSTANT / (r * r));

        let coriolis_accel = if params.coriolis_enabled {
            -2.0 * EARTH_OMEGA.cross(vel)
        } else {
            DVec3::ZERO
        };

        let centrifugal_accel = if params.centrifugal_enabled {
            -EARTH_OMEGA.cross(EARTH_OMEGA.cross(pos))
        } else {
            DVec3::ZERO
        };

        let (model_accel, new_phase, mass_delta) =
            model.compute_acceleration(pos, vel, mass, timer, dt);
        _phase = new_phase;
        mass += mass_delta;
        timer += dt;

        let total_accel = gravity_accel + coriolis_accel + centrifugal_accel + model_accel;
        vel += total_accel * dt as f64;
        pos += vel * dt as f64;
    }

    let (lat, lon) = ecef_to_geodetic_deg(pos);
    let miss = haversine_m(lat, lon, tgt_lat, tgt_lon);
    GroundImpactResult {
        landed: false,
        flight_time_s: timer,
        position_ecef: pos,
        impact_lat_deg: lat,
        impact_lon_deg: lon,
        miss_distance_m: miss,
    }
}
