use bevy::prelude::*;
use glam::DVec3;
use serde::{Deserialize, Serialize};
use crate::*;

// ============================================================================
// Core Physical Constants
// ============================================================================

// --- Earth & Atmospheric Models ---
/// Mean radius of the Earth in meters.
/// Used for altitude calculations and spherical coordinate transformations.
/// [Reference: WGS 84 Reference Ellipsoid / Mean Radius]
pub const EARTH_RADIUS: f64 = 6_371_000.0;
/// Specific gas constant for dry air (J/(kg·K)).
/// Used in the Ideal Gas Law: P = rho * R * T.
/// [Reference: International Standard Atmosphere (ISA)]
pub const R_AIR: f64 = 287.05;
/// Heat capacity ratio (adiabatic index) for diatomic gas (dry air).
/// Used to calculate the speed of sound: a = sqrt(gamma * R * T).
pub const GAMMA: f64 = 1.4;
/// Standard sea level temperature in Kelvin (15°C).
/// Base value for ISA troposphere gradient calculations.
pub const T0: f64 = 288.15;
/// Standard atmospheric pressure at sea level in Pascals.
pub const P0: f64 = 101325.0;
/// Standard gravity at sea level (m/s^2).
/// Used for Specific Impulse (Isp) calculations: Thrust = Isp * burn_rate * G0.
pub const G0: f64 = 9.80665;

// --- Orbital Mechanics ---
/// Universal Gravitational Constant (m^3 * kg^-1 * s^-2).
/// [Reference: Newton's Law of Universal Gravitation]
pub const G: f64 = 6.67430e-11;
/// Mass of the Earth in kilograms.
pub const EARTH_MASS: f64 = 5.972e24;
/// Standard gravitational parameter (mu = G * M).
/// Used to calculate the gravitational force vector: Fg = -mu * r / |r|^3.
pub const GRAVITY_CONSTANT: f64 = G * EARTH_MASS;
/// Earth's angular velocity vector in radians per second.
/// Assumes rotation around the Y-axis. Handedness: +Y is North Pole.
/// Used for Coriolis and Centrifugal force calculations.
/// [Reference: Earth's rotation period (Sidereal day)]
pub const EARTH_OMEGA: DVec3 = DVec3::new(0.0, 7.2921159e-5, 0.0); // Rad/s around Y axis

// Target Coordinates
pub const MOSCOW_LAT: f64 = 55.7558;
pub const MOSCOW_LON: f64 = 37.6173;

// --- Core Structs ---
// --- Missile Specification Structure ---

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
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

pub fn get_missile_registry() -> Vec<MissileSpecs> {
    let json_str = std::fs::read_to_string("assets/missiles.json")
        .expect("⚠️ Could not read assets/missiles.json");
    let static_str: &'static str = Box::leak(json_str.into_boxed_str());
    serde_json::from_str(static_str)
        .expect("⚠️ Failed to parse assets/missiles.json")
}

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
    fn specs(&self) -> &MissileSpecs;
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
    fn specs(&self) -> &MissileSpecs { &self.specs }

    fn compute_acceleration(
        &self, 
        position: DVec3, 
        velocity: DVec3, 
        mass: f64, 
        timer: f32,
        dt: f32
    ) -> (DVec3, FlightPhase, f64) {
        
        // --- 1. Environmental Properties ---
        // Altitude = distance from Earth center minus Earth radius.
        let r = position.length();
        let altitude = (r - EARTH_RADIUS).max(0.0);
        
        // Fetch atmospheric conditions based on the standard ISA model.
        let (rho, sound_speed, pressure) = get_isa_properties(altitude);
        let speed = velocity.length();
        // Mach number = object speed / speed of sound in local medium.
        let mach = speed / sound_speed;

        // --- 2. Flight Phase Determination ---
        let phase = if timer < self.specs.burn_time {
            FlightPhase::Boost
        } else if altitude > 120000.0 || velocity.dot(position) >= 0.0 {
            // Ballistic phase applies when out of the atmosphere (120km, Karman line+) 
            // or while still moving upwards (dot product of velocity and position is positive).
            FlightPhase::Ballistic
        } else {
            // Re-entry begins when below 120km and descending.
            FlightPhase::ReEntry
        };

        // --- 3. Thrust Calculations ---
        let mut thrust_accel = DVec3::ZERO;
        let mut mass_delta = 0.0;
        
        if phase == FlightPhase::Boost {
            let target_dir = (self.target_ecef - position).normalize();
            let up = position.normalize(); // Gravity turn initial vector
            
            // Pitch factor controls the gravity turn profile over time.
            let pitch_factor = if timer < self.specs.pitch_start_time {
                0.0
            } else {
                ((timer - self.specs.pitch_start_time) / (self.specs.burn_time - self.specs.pitch_start_time)).min(1.0) as f64
            };
            
            // Blends the "up" vector with the "target" direction to simulate pitching over.
            let current_thrust_dir = (up * (self.specs.boost_loft_factor - pitch_factor * self.specs.pitch_turn_rate) 
                + target_dir * (pitch_factor * self.specs.pitch_turn_rate)).normalize();
            
            // Dynamic Specific Impulse (Isp) based on atmospheric pressure.
            // Rocket engines become more efficient in a vacuum as exhaust expansion is unimpeded.
            // Equation: Isp_current = Isp_vac - (Isp_vac - Isp_sl) * (P_current / P0)
            // [Reference: https://en.wikipedia.org/wiki/Specific_impulse#Altitude_dependence]
            let p_ratio = (pressure / P0).clamp(0.0, 1.0);
            let current_isp = self.specs.isp_vacuum - (self.specs.isp_vacuum - self.specs.isp_sea_level) * p_ratio;
            
            let burn_rate = self.specs.burn_rate();
            // Tsiolkovsky Rocket Equation component: Thrust = m_dot * Isp * g0
            let thrust_force = burn_rate * current_isp * G0;

            // Newton's Second Law: a = F/m
            thrust_accel = current_thrust_dir * (thrust_force / mass);
            
            // Mass decreases over time by the burn rate (kg/s).
            mass_delta = -burn_rate * dt as f64;
        }

        // --- 4. Aerodynamic Drag Calculations ---
        // Drag equation: F_D = 0.5 * rho * v^2 * Cd * A
        // [Reference: https://en.wikipedia.org/wiki/Drag_equation]
        let cd = get_mach_drag(mach);
        let drag_force = -0.5 * rho * (speed * speed) * cd * self.specs.area;
        let drag_accel = if speed > 1e-3 {
            // Drag vector opposes the velocity vector.
            (velocity.normalize() * drag_force) / mass
        } else {
            DVec3::ZERO
        };

        // Output cumulative non-gravitational accelerations.
        // Note: Gravity and non-inertial frame forces (Coriolis, Centrifugal) 
        // are computed globally in the `physics_system`.
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
// ============================================================================
// Atmospheric and Aerodynamic Models
// ============================================================================

/// Computes air density, speed of sound, and pressure at a given altitude
/// using the International Standard Atmosphere (ISA) model.
/// 
/// The atmosphere is divided into layers with distinct temperature lapse rates.
/// [Reference: https://en.wikipedia.org/wiki/International_Standard_Atmosphere]
pub fn get_isa_properties(altitude: f64) -> (f64, f64, f64) {
    let (t, p) = if altitude < 11000.0 {
        // Troposphere: constant temperature lapse rate (-6.5 K/km).
        // P = P0 * (T/T0)^(g0/(a*R))
        let t = T0 - 0.0065 * altitude;
        let p = P0 * (t / T0).powf(G0 / (0.0065 * R_AIR));
        (t, p)
    } else if altitude < 20000.0 {
        // Tropopause: constant temperature layer.
        // P = P11 * exp(-g0 * (h - h11) / (R * T))
        let t = 216.65;
        let p_11 = P0 * (216.65 / T0).powf(G0 / (0.0065 * R_AIR));
        let p = p_11 * f64::exp(-G0 * (altitude - 11000.0) / (R_AIR * t));
        (t, p)
    } else if altitude < 32000.0 {
        // Stratosphere (lower): positive lapse rate (+1.0 K/km).
        let t = 216.65 + 0.001 * (altitude - 20000.0);
        let p_20 = 22632.0;
        let p = p_20 * (t / 216.65).powf(-G0 / (0.001 * R_AIR));
        (t, p)
    } else {
        // Upper stratosphere & beyond (Simplified exponential decay extension).
        // Barometric formula mapping: density exponentially decays with scale height (~8.5km).
        let t = 228.65;
        let rho = 1.225 * f64::exp(-altitude / 8500.0);
        let p = rho * R_AIR * t;
        // Density = rho, Speed of Sound a = sqrt(gamma * R * T), Pressure = p
        return (rho, (GAMMA * R_AIR * t).sqrt(), p);
    };
    
    // Ideal Gas Law: rho = P / (R * T)
    let rho = p / (R_AIR * t);
    let sound_speed = (GAMMA * R_AIR * t).sqrt();
    (rho, sound_speed, p)
}

/// Empirical Drag Coefficient curve relative to Mach number.
/// Accurately scales wave drag, which spikes dramatically as the missile 
/// enters the transonic regime (Mach 1.0) and slowly decays in hypersonic phases.
/// 
/// [Reference: Wave Drag Profile modeling - https://en.wikipedia.org/wiki/Wave_drag]
pub fn get_mach_drag(mach: f64) -> f64 {
    if mach < 0.8 {
        0.15
    } else if mach < 1.2 {
        0.15 + 0.35 * (mach - 0.8) / 0.4
    } else if mach < 2.0 {
        0.5 * (1.2 / mach).powf(0.5)
    } else {
        (0.4 - (mach - 2.0) * 0.05).max(0.3)
    }
}
