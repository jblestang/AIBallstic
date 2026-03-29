//! Missile data, boost guidance, and integration-facing vehicle physics (`BallisticMissilePhysics`).
//!
//! Core **environment kernels** (gravity, atmosphere, Earth frame, propulsion formulas) live in
//! [`crate::physics`]. This module re-exports them for existing `use aiballistic::missiles::*` call sites.

use bevy::prelude::*;
use glam::DVec3;
use serde::{Deserialize, Serialize};

pub use crate::physics::{
    ecef_to_geodetic_deg, ecef_to_geodetic_deg_h, ecef_to_geodetic_rad_h, ellipsoid_height_m,
    geodetic_to_ecef, get_isa_properties, get_mach_drag, gravity_acceleration_ecef, mass_flow_kg_s,
    rocket_thrust_newton, EARTH_J2, EARTH_J3, EARTH_J4, EARTH_MASS, EARTH_OMEGA, EARTH_RADIUS,
    G, G0, GAMMA, GRAVITY_CONSTANT, P0, R_AIR, T0, WGS84_A, WGS84_B, WGS84_E2, WGS84_EP2, WGS84_F,
    WGS84_INV_F,
};

// Reference points (ABM scenario still anchored near Moscow)
pub const MOSCOW_LAT: f64 = 55.7558;
pub const MOSCOW_LON: f64 = 37.6173;

pub const DIEGO_GARCIA_LAT: f64 = -7.31337;
pub const DIEGO_GARCIA_LON: f64 = 72.41615;

pub const TEHRAN_LAT: f64 = 35.6892;
pub const TEHRAN_LON: f64 = 51.3890;

pub const PARIS_LAT: f64 = 48.8566;
pub const PARIS_LON: f64 = 2.3522;

/// Launch altitude (m) for scenario presets (slightly above terrain).
pub const SCENARIO_LAUNCH_ALT_M: f64 = 10.0;

/// Named sites used for launch / aim point selection in the UI.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub enum GeoSiteId {
    #[default]
    Tehran,
    Paris,
    Moscow,
    DiegoGarcia,
}

impl GeoSiteId {
    pub const ALL: [GeoSiteId; 4] = [
        GeoSiteId::Tehran,
        GeoSiteId::Paris,
        GeoSiteId::Moscow,
        GeoSiteId::DiegoGarcia,
    ];

    pub fn label(self) -> &'static str {
        match self {
            GeoSiteId::Tehran => "Tehran",
            GeoSiteId::Paris => "Paris",
            GeoSiteId::Moscow => "Moscow",
            GeoSiteId::DiegoGarcia => "Diego Garcia",
        }
    }

    pub fn lat_lon(self) -> (f64, f64) {
        match self {
            GeoSiteId::Tehran => (TEHRAN_LAT, TEHRAN_LON),
            GeoSiteId::Paris => (PARIS_LAT, PARIS_LON),
            GeoSiteId::Moscow => (MOSCOW_LAT, MOSCOW_LON),
            GeoSiteId::DiegoGarcia => (DIEGO_GARCIA_LAT, DIEGO_GARCIA_LON),
        }
    }

    pub fn launch_ecef(self) -> DVec3 {
        let (lat, lon) = self.lat_lon();
        geodetic_to_ecef(lat, lon, SCENARIO_LAUNCH_ALT_M)
    }

    pub fn aim_ecef(self) -> DVec3 {
        let (lat, lon) = self.lat_lon();
        geodetic_to_ecef(lat, lon, 0.0)
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct StageSpecs {
    pub dry_mass: f64,
    pub fuel_mass: f64,
    pub burn_time: f32,
    pub isp_sea_level: f64,
    pub isp_vacuum: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BoostPitchConfig {
    pub pitch_start_time: f32,
    pub pitch_exponent: f64,
    pub initial_elevation_deg: f64,
    pub end_elevation_deg: f64,
}

impl Default for BoostPitchConfig {
    fn default() -> Self {
        Self {
            pitch_start_time: 8.0,
            pitch_exponent: 1.0,
            initial_elevation_deg: 45.0,
            end_elevation_deg: 30.0,
        }
    }
}

impl BoostPitchConfig {
    pub fn pitch_progress(&self, total_burn_time: f32, timer: f32) -> f64 {
        boost_pitch_progress(
            total_burn_time,
            self.pitch_start_time,
            self.pitch_exponent,
            timer,
        )
    }

    pub fn thrust_elevation_deg_at(&self, total_burn_time: f32, timer: f32) -> f64 {
        if timer < self.pitch_start_time {
            return 90.0;
        }
        let pf = self.pitch_progress(total_burn_time, timer);
        thrust_elevation_deg_interpolated(
            self.initial_elevation_deg,
            self.end_elevation_deg,
            pf,
        )
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissileSpecs {
    pub name: &'static str,
    pub dry_mass: f64,
    pub area: f64,
    pub stages: Vec<StageSpecs>,
    #[serde(default)]
    pub mirv: Option<MirvConfig>,
    #[serde(default)]
    pub url: Vec<String>,
    #[serde(default)]
    pub quality: Option<String>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct MirvConfig {
    pub num_warheads: u32,
    pub warhead_mass: f64,
    pub warhead_area: f64,
    pub num_decoys: u32,
    pub decoy_mass: f64,
    pub decoy_area: f64,
    pub deploy_delay: f32,
    pub spread_velocity: f64,
}

impl MissileSpecs {
    pub fn is_multistage(&self) -> bool {
        self.stages.len() > 1
    }

    pub fn total_mass(&self) -> f64 {
        self.dry_mass + self.stages.iter().map(|s| s.dry_mass + s.fuel_mass).sum::<f64>()
    }

    pub fn total_dry_mass(&self) -> f64 {
        self.dry_mass + self.stages.iter().map(|s| s.dry_mass).sum::<f64>()
    }

    pub fn total_burn_time(&self) -> f32 {
        self.stages.iter().map(|s| s.burn_time).sum()
    }

    pub fn burn_rate(&self) -> f64 {
        self.stages
            .first()
            .map(|s| s.fuel_mass / s.burn_time as f64)
            .unwrap_or(0.0)
    }

    pub fn active_stage(&self, timer: f32) -> Option<(usize, &StageSpecs, f32)> {
        let mut elapsed = 0.0f32;
        for (i, stage) in self.stages.iter().enumerate() {
            if timer < elapsed + stage.burn_time {
                return Some((i, stage, timer - elapsed));
            }
            elapsed += stage.burn_time;
        }
        None
    }

    pub fn stage_end_time(&self, stage_idx: usize) -> f32 {
        self.stages[..=stage_idx].iter().map(|s| s.burn_time).sum()
    }
}

pub fn boost_pitch_progress(
    total_burn_time: f32,
    pitch_start_time: f32,
    pitch_exponent: f64,
    timer: f32,
) -> f64 {
    if timer < pitch_start_time {
        return 0.0;
    }
    let denom = (total_burn_time - pitch_start_time) as f64;
    if denom <= 1e-9 {
        return 1.0;
    }
    let linear = ((timer - pitch_start_time) as f64 / denom).min(1.0);
    linear.powf(pitch_exponent)
}

#[inline]
pub fn thrust_elevation_deg_interpolated(
    initial_elevation_deg: f64,
    end_elevation_deg: f64,
    pitch_progress: f64,
) -> f64 {
    let t0 = initial_elevation_deg.to_radians();
    let t1 = end_elevation_deg.to_radians();
    let w = pitch_progress.clamp(0.0, 1.0);
    let theta = t0 + (t1 - t0) * w;
    theta.to_degrees()
}

pub fn get_missile_registry() -> Vec<MissileSpecs> {
    #[cfg(target_arch = "wasm32")]
    let json_str: &'static str = include_str!("../assets/missiles.json");
    #[cfg(not(target_arch = "wasm32"))]
    let json_str: &'static str = {
        let s = std::fs::read_to_string("assets/missiles.json")
            .expect("⚠️ Could not read assets/missiles.json");
        Box::leak(s.into_boxed_str())
    };
    serde_json::from_str(json_str).expect("⚠️ Failed to parse assets/missiles.json")
}

#[derive(Debug, Clone, Copy, PartialEq, Reflect)]
pub enum FlightPhase {
    Boost,
    Ballistic,
    ReEntry,
    Landed,
}

fn stage_jettison_delta(specs: &MissileSpecs, t0: f32, t1: f32) -> f64 {
    if t1 <= t0 {
        return 0.0;
    }
    let mut d = 0.0;
    for (i, stage) in specs.stages.iter().enumerate() {
        let end_t = specs.stage_end_time(i);
        if t0 < end_t && end_t <= t1 {
            d -= stage.dry_mass;
        }
    }
    d
}

#[inline]
pub fn flight_phase_from_state(
    timer: f32,
    total_bt: f32,
    altitude: f64,
    position: DVec3,
    velocity: DVec3,
) -> FlightPhase {
    if timer < total_bt {
        FlightPhase::Boost
    } else if altitude > 120_000.0 || velocity.dot(position) >= 0.0 {
        FlightPhase::Ballistic
    } else {
        FlightPhase::ReEntry
    }
}

pub trait PhysicsModel: Send + Sync {
    fn name(&self) -> &str;
    fn specs(&self) -> &MissileSpecs;
    fn compute_acceleration(
        &self,
        position: DVec3,
        velocity: DVec3,
        mass: f64,
        timer: f32,
        dt: f32,
    ) -> (DVec3, FlightPhase, f64);
    fn get_stats(&self, position: DVec3, velocity: DVec3, timer: f32) -> Vec<(String, String)>;

    /// Dormand–Prince (DOPRI5) sub-step: **r**, **v**, **m**, simulation time in rotating ECEF.
    fn dopri5_substep(
        &self,
        pos: &mut DVec3,
        vel: &mut DVec3,
        mass: &mut f64,
        timer: &mut f32,
        h: f32,
        coriolis: bool,
        centrifugal: bool,
    ) -> FlightPhase;
}

pub struct BallisticMissilePhysics {
    pub specs: MissileSpecs,
    pub target_ecef: DVec3,
    pub pitch: BoostPitchConfig,
}

impl BallisticMissilePhysics {
    pub fn new(specs: MissileSpecs, target_ecef: DVec3, pitch: BoostPitchConfig) -> Self {
        Self {
            specs,
            target_ecef,
            pitch,
        }
    }

    /// Thrust + drag; **dm/dt** = −ṁ en boost (séparations d’étage discrètes appliquées après le pas RK).
    pub fn vehicle_acceleration_and_dm_dt(
        &self,
        position: DVec3,
        velocity: DVec3,
        mass: f64,
        timer: f32,
    ) -> (DVec3, f64, FlightPhase) {
        let mass = mass.max(1.0);
        let altitude = ellipsoid_height_m(position).max(0.0);
        let (rho, sound_speed, pressure) = get_isa_properties(altitude);
        let speed = velocity.length();
        let mach = speed / sound_speed;

        let total_bt = self.specs.total_burn_time();
        let phase = flight_phase_from_state(timer, total_bt, altitude, position, velocity);

        let mut thrust_accel = DVec3::ZERO;
        let mut dm_dt = 0.0;

        if phase == FlightPhase::Boost {
            let up = position.normalize();
            let target_vec = self.target_ecef - position;
            let horizontal_dir = (target_vec - up * up.dot(target_vec)).normalize();
            let theta_rad = self.pitch.thrust_elevation_deg_at(total_bt, timer).to_radians();
            let current_thrust_dir =
                (up * theta_rad.sin() + horizontal_dir * theta_rad.cos()).normalize();

            if let Some((_stage_idx, stage, t_in_stage)) = self.specs.active_stage(timer) {
                let (mdot, mdot_ref) =
                    mass_flow_kg_s(stage.fuel_mass, stage.burn_time, t_in_stage);
                let thrust_force = rocket_thrust_newton(
                    mdot,
                    mdot_ref,
                    stage.isp_vacuum,
                    stage.isp_sea_level,
                    pressure,
                );
                thrust_accel = current_thrust_dir * (thrust_force / mass);
                dm_dt = -mdot;
            }
        }

        let cd = get_mach_drag(mach);
        let drag_force = -0.5 * rho * (speed * speed) * cd * self.specs.area;
        let drag_accel = if speed > 1e-3 {
            (velocity.normalize() * drag_force) / mass
        } else {
            DVec3::ZERO
        };

        (thrust_accel + drag_accel, dm_dt, phase)
    }
}

impl PhysicsModel for BallisticMissilePhysics {
    fn name(&self) -> &str {
        self.specs.name
    }
    fn specs(&self) -> &MissileSpecs {
        &self.specs
    }

    fn compute_acceleration(
        &self,
        position: DVec3,
        velocity: DVec3,
        mass: f64,
        timer: f32,
        dt: f32,
    ) -> (DVec3, FlightPhase, f64) {
        let (a, dm_dt, phase) = self.vehicle_acceleration_and_dm_dt(position, velocity, mass, timer);
        let mut mass_delta = dm_dt * dt as f64;
        if phase == FlightPhase::Boost {
            mass_delta += stage_jettison_delta(&self.specs, timer, timer + dt);
        }
        (a, phase, mass_delta)
    }

    fn dopri5_substep(
        &self,
        pos: &mut DVec3,
        vel: &mut DVec3,
        mass: &mut f64,
        timer: &mut f32,
        h: f32,
        coriolis: bool,
        centrifugal: bool,
    ) -> FlightPhase {
        use crate::physics::{dopri5_step_7, inertial_accelerations};

        let t0 = *timer;
        let mut y = [
            pos.x,
            pos.y,
            pos.z,
            vel.x,
            vel.y,
            vel.z,
            mass.max(1.0),
        ];
        let slf = self;
        dopri5_step_7(&mut y, t0, h, |t, state| {
            let p = DVec3::new(state[0], state[1], state[2]);
            let v = DVec3::new(state[3], state[4], state[5]);
            let m = state[6].max(1.0);
            let a_in = inertial_accelerations(p, v, coriolis, centrifugal);
            let (a_v, dm_dt, _) = slf.vehicle_acceleration_and_dm_dt(p, v, m, t);
            let a = a_in + a_v;
            [v.x, v.y, v.z, a.x, a.y, a.z, dm_dt]
        });

        *pos = DVec3::new(y[0], y[1], y[2]);
        *vel = DVec3::new(y[3], y[4], y[5]);
        *mass = y[6].max(1.0);
        *timer = t0 + h;
        *mass += stage_jettison_delta(&self.specs, t0, *timer);
        *mass = (*mass).max(1.0);

        let altitude = ellipsoid_height_m(*pos).max(0.0);
        flight_phase_from_state(
            *timer,
            self.specs.total_burn_time(),
            altitude,
            *pos,
            *vel,
        )
    }

    fn get_stats(&self, position: DVec3, velocity: DVec3, timer: f32) -> Vec<(String, String)> {
        let altitude = ellipsoid_height_m(position).max(0.0);
        let (_rho, sound_speed, pressure) = get_isa_properties(altitude);
        let speed = velocity.length();
        let mach = speed / sound_speed;
        let p_ratio = (pressure / P0).clamp(0.0, 1.0);

        let total_bt = self.specs.total_burn_time();
        let (current_isp_str, thrust_kn_str, p_ratio_str, stage_str) = if timer < total_bt {
            if let Some((idx, stage, t_in)) = self.specs.active_stage(timer) {
                let (mdot, mdot_ref) = mass_flow_kg_s(stage.fuel_mass, stage.burn_time, t_in);
                let f_n = rocket_thrust_newton(
                    mdot,
                    mdot_ref,
                    stage.isp_vacuum,
                    stage.isp_sea_level,
                    pressure,
                );
                let isp_eff = if mdot > 1e-9 {
                    f_n / (mdot * G0)
                } else {
                    0.0
                };
                (
                    format!("{:.1} s", isp_eff),
                    format!("{:.1}", f_n * 1e-3),
                    format!("{:.4}", p_ratio),
                    format!("{}/{}", idx + 1, self.specs.stages.len()),
                )
            } else {
                ("N/A".into(), "N/A".into(), "N/A".into(), "N/A".into())
            }
        } else {
            ("N/A".into(), "N/A".into(), "N/A".into(), "N/A".into())
        };

        let mut stats = vec![
            ("Altitude".into(), format!("{:.2} km", altitude / 1000.0)),
            ("Mach".into(), format!("{:.2}", mach)),
            ("Speed".into(), format!("{:.0} m/s", speed)),
            ("Isp eff".into(), current_isp_str),
            (
                "Thrust".into(),
                if thrust_kn_str == "N/A" {
                    "N/A".into()
                } else {
                    format!("{} kN", thrust_kn_str)
                },
            ),
            ("p_a / P0".into(), p_ratio_str),
        ];
        if !self.specs.stages.is_empty() {
            stats.push(("Stage".into(), stage_str));
        }
        if timer < total_bt {
            let elev = self.pitch.thrust_elevation_deg_at(total_bt, timer);
            stats.push(("Thrust elev.".into(), format!("{:.1}° (horiz.)", elev)));
        }
        stats
    }
}
