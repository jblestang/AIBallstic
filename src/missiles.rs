use bevy::prelude::*;
use glam::DVec3;
use crate::*;

// --- Missile Specification Structure ---

#[derive(Debug, Clone, Copy, Reflect)]
pub struct MissileSpecs {
    pub name: &'static str,
    pub dry_mass: f64,
    pub fuel_mass: f64,
    pub burn_time: f32,
    pub area: f64,
    pub isp_sea_level: f64,
    pub isp_vacuum: f64,
    pub pitch_start_time: f32,
    pub pitch_turn_rate: f64, // Factor for mixing target dir
    pub boost_loft_factor: f64, // Elevation bias during boost
}

impl MissileSpecs {
    pub fn burn_rate(&self) -> f64 {
        self.fuel_mass / self.burn_time as f64
    }
}

// --- Predefined Missile Specs ---

pub const SCUD_C_SPECS: MissileSpecs = MissileSpecs {
    name: "Scud-C",
    dry_mass: 1500.0,
    fuel_mass: 4500.0,
    burn_time: 70.0,
    area: 0.6,
    isp_sea_level: 225.0,
    isp_vacuum: 250.0,
    pitch_start_time: 5.0,
    pitch_turn_rate: 0.5,
    boost_loft_factor: 1.0,
};

pub const MRBM_SPECS: MissileSpecs = MissileSpecs {
    name: "Shahab-3 (MRBM)",
    dry_mass: 2000.0,
    fuel_mass: 10000.0,
    burn_time: 110.0,
    area: 0.8,
    isp_sea_level: 235.0,
    isp_vacuum: 280.0,
    pitch_start_time: 10.0,
    pitch_turn_rate: 0.7,
    boost_loft_factor: 1.1,
};

pub const GHADR_SPECS: MissileSpecs = MissileSpecs {
    name: "Ghadr-110",
    dry_mass: 1800.0,
    fuel_mass: 12000.0,
    burn_time: 125.0,
    area: 0.75,
    isp_sea_level: 240.0,
    isp_vacuum: 295.0,
    pitch_start_time: 12.0,
    pitch_turn_rate: 0.75,
    boost_loft_factor: 1.15,
};

// --- Trait & Enum ---

#[derive(Debug, Clone, Copy, PartialEq, Reflect)]
pub enum FlightPhase {
    Boost,
    Ballistic,
    ReEntry,
    Landed,
}

pub trait PhysicsModel: Send + Sync {
    fn name(&self) -> &str;
    fn compute_acceleration(
        &self, 
        position: DVec3, 
        velocity: DVec3, 
        mass: f64, 
        timer: f32,
        dt: f32
    ) -> (DVec3, FlightPhase, f64);
    fn get_stats(&self, position: DVec3, velocity: DVec3, timer: f32) -> Vec<(String, String)>;
}

// --- Unified Implementation ---

pub struct BallisticMissilePhysics {
    pub specs: MissileSpecs,
    pub target_ecef: DVec3,
}

impl BallisticMissilePhysics {
    pub fn new(specs: MissileSpecs) -> Self {
        Self {
            specs,
            target_ecef: geodetic_to_ecef(MOSCOW_LAT, MOSCOW_LON, 0.0),
        }
    }
}

impl PhysicsModel for BallisticMissilePhysics {
    fn name(&self) -> &str { self.specs.name }

    fn compute_acceleration(
        &self, 
        position: DVec3, 
        velocity: DVec3, 
        mass: f64, 
        timer: f32,
        dt: f32
    ) -> (DVec3, FlightPhase, f64) {
        let r = position.length();
        let altitude = (r - EARTH_RADIUS).max(0.0);
        let (rho, sound_speed, pressure) = get_isa_properties(altitude);
        let speed = velocity.length();
        let mach = speed / sound_speed;

        let phase = if timer < self.specs.burn_time {
            FlightPhase::Boost
        } else if altitude > 40000.0 || velocity.dot(position) >= 0.0 {
            FlightPhase::Ballistic
        } else {
            FlightPhase::ReEntry
        };

        let mut thrust_accel = DVec3::ZERO;
        let mut mass_delta = 0.0;
        if phase == FlightPhase::Boost {
            let target_dir = (self.target_ecef - position).normalize();
            let up = position.normalize();
            
            let pitch_factor = if timer < self.specs.pitch_start_time {
                0.0
            } else {
                ((timer - self.specs.pitch_start_time) / (self.specs.burn_time - self.specs.pitch_start_time)).min(1.0) as f64
            };
            
            let current_thrust_dir = (up * (self.specs.boost_loft_factor - pitch_factor * self.specs.pitch_turn_rate) 
                + target_dir * (pitch_factor * self.specs.pitch_turn_rate)).normalize();
            
            let p_ratio = (pressure / P0).clamp(0.0, 1.0);
            let current_isp = self.specs.isp_vacuum - (self.specs.isp_vacuum - self.specs.isp_sea_level) * p_ratio;
            let burn_rate = self.specs.burn_rate();
            let thrust_force = burn_rate * current_isp * G0;

            thrust_accel = current_thrust_dir * (thrust_force / mass);
            mass_delta = -burn_rate * dt as f64;
        }

        let cd = get_mach_drag(mach);
        let drag_force = -0.5 * rho * (speed * speed) * cd * self.specs.area;
        let drag_accel = if speed > 1e-3 {
            (velocity.normalize() * drag_force) / mass
        } else {
            DVec3::ZERO
        };

        (thrust_accel + drag_accel, phase, mass_delta)
    }

    fn get_stats(&self, position: DVec3, velocity: DVec3, timer: f32) -> Vec<(String, String)> {
        let altitude = (position.length() - EARTH_RADIUS).max(0.0);
        let (_rho, sound_speed, pressure) = get_isa_properties(altitude);
        let speed = velocity.length();
        let mach = speed / sound_speed;
        let p_ratio = (pressure / P0).clamp(0.0, 1.0);
        let (current_isp_str, p_ratio_str) = if timer < self.specs.burn_time {
            let current_isp = self.specs.isp_vacuum - (self.specs.isp_vacuum - self.specs.isp_sea_level) * p_ratio;
            (format!("{:.1} s", current_isp), format!("{:.4}", p_ratio))
        } else {
            ("N/A".into(), "N/A".into())
        };

        vec![
            ("Altitude".into(), format!("{:.2} km", altitude / 1000.0)),
            ("Mach".into(), format!("{:.2}", mach)),
            ("Speed".into(), format!("{:.0} m/s", speed)),
            ("Isp".into(), current_isp_str),
            ("Pressure Ratio".into(), p_ratio_str),
        ]
    }
}
