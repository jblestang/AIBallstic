//! Headless trajectory integration matching `physics_system` in `main.rs` (fixed timestep).

use glam::DVec3;

use crate::missiles::{
    BallisticMissilePhysics, BoostPitchConfig, FlightPhase, MissileSpecs, PhysicsModel,
};
use crate::physics::{ecef_to_geodetic_deg, ellipsoid_height_m, EARTH_RADIUS};

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
    /// Last integrated position (may be slightly below the ellipsoid when `landed`).
    pub position_ecef: DVec3,
    pub impact_lat_deg: f64,
    pub impact_lon_deg: f64,
    /// Great-circle miss distance to the aim point used in `BallisticMissilePhysics`.
    pub miss_distance_m: f64,
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

/// Integrate until ellipsoid height ≤ 0 (WGS84 ground) after 1 s, or `max_time_s`.
pub fn simulate_until_ground_impact(
    specs: MissileSpecs,
    launch_ecef: DVec3,
    target_ecef: DVec3,
    pitch: &BoostPitchConfig,
    params: &SimParams,
) -> GroundImpactResult {
    let model = BallisticMissilePhysics::new(specs, target_ecef, *pitch);
    let (tgt_lat, tgt_lon) = ecef_to_geodetic_deg(target_ecef);
    let dt = params.dt as f32;

    let mut pos = launch_ecef;
    let mut vel = DVec3::ZERO;
    let mut mass = model.specs().total_mass();
    let mut timer = 0.0f32;
    let mut _phase = FlightPhase::Boost;

    while timer < params.max_time_s {
        if ellipsoid_height_m(pos) <= 0.0 && timer > 1.0 {
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

        _phase = model.dopri5_substep(
            &mut pos,
            &mut vel,
            &mut mass,
            &mut timer,
            dt,
            params.coriolis_enabled,
            params.centrifugal_enabled,
        );
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

/// Result of integrating a full trajectory for analysis (nomograms, etc.).
#[derive(Debug, Clone)]
pub struct TrajectoryMetrics {
    pub landed: bool,
    /// Great-circle distance from launch point to impact (km).
    pub ground_range_km: f64,
    /// Maximum WGS84 ellipsoidal altitude along the path (km).
    pub apogee_km: f64,
    pub flight_time_s: f32,
}

/// Faster integration for batch scans (abbaques); still matches `simulate_until_ground_impact` physics.
pub fn nomogram_sim_params() -> SimParams {
    SimParams {
        dt: 0.025,
        coriolis_enabled: true,
        centrifugal_enabled: true,
        max_time_s: 12_000.0,
    }
}

/// Integrate trajectory; track apogee and ground range (launch → impact). Same forces as `simulate_until_ground_impact`.
pub fn simulate_trajectory_metrics(
    specs: MissileSpecs,
    launch_ecef: DVec3,
    target_ecef: DVec3,
    pitch: &BoostPitchConfig,
    params: &SimParams,
) -> TrajectoryMetrics {
    let (launch_lat, launch_lon) = ecef_to_geodetic_deg(launch_ecef);
    let model = BallisticMissilePhysics::new(specs, target_ecef, *pitch);
    let dt = params.dt as f32;

    let mut pos = launch_ecef;
    let mut vel = DVec3::ZERO;
    let mut mass = model.specs().total_mass();
    let mut timer = 0.0f32;
    let mut max_h = 0.0f64;

    while timer < params.max_time_s {
        max_h = max_h.max(ellipsoid_height_m(pos));

        if ellipsoid_height_m(pos) <= 0.0 && timer > 1.0 {
            let (lat, lon) = ecef_to_geodetic_deg(pos);
            let range_m = haversine_m(launch_lat, launch_lon, lat, lon);
            return TrajectoryMetrics {
                landed: true,
                ground_range_km: range_m / 1000.0,
                apogee_km: max_h / 1000.0,
                flight_time_s: timer,
            };
        }

        let _phase = model.dopri5_substep(
            &mut pos,
            &mut vel,
            &mut mass,
            &mut timer,
            dt,
            params.coriolis_enabled,
            params.centrifugal_enabled,
        );
        max_h = max_h.max(ellipsoid_height_m(pos));
    }

    max_h = max_h.max(ellipsoid_height_m(pos));
    let (lat, lon) = ecef_to_geodetic_deg(pos);
    let range_m = haversine_m(launch_lat, launch_lon, lat, lon);
    TrajectoryMetrics {
        landed: false,
        ground_range_km: range_m / 1000.0,
        apogee_km: max_h / 1000.0,
        flight_time_s: timer,
    }
}
