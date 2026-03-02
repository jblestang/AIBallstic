use bevy::prelude::*;
use glam::DVec3;
use serde::{Deserialize, Serialize};
use crate::*;

// --- Core Structs ---
// --- Physical Constants (ISA & Earth) ---
pub const EARTH_RADIUS: f64 = 6_371_000.0;
pub const R_AIR: f64 = 287.05;
pub const GAMMA: f64 = 1.4;
pub const T0: f64 = 288.15;
pub const P0: f64 = 101325.0;
pub const G0: f64 = 9.80665;

// Gravity & Rotation
pub const G: f64 = 6.67430e-11;
pub const EARTH_MASS: f64 = 5.972e24;
pub const GRAVITY_CONSTANT: f64 = G * EARTH_MASS;
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

// Recent Iranian Models

pub const KHORRAMSHAHR_4_SPECS: MissileSpecs = MissileSpecs {
    name: "Khorramshahr-4 (Kheibar)",
    dry_mass: 3500.0, // Solid heavy structure for 1.5t warhead
    fuel_mass: 25000.0, 
    burn_time: 145.0, // Liquid fueled sustainer
    area: 1.77, // 1.5m diameter
    isp_sea_level: 265.0,
    isp_vacuum: 310.0,
    pitch_start_time: 15.0,
    pitch_turn_rate: 0.8,
    boost_loft_factor: 1.2,
};

pub const SEJJIL_2_SPECS: MissileSpecs = MissileSpecs {
    name: "Sejjil-2",
    dry_mass: 2500.0,
    fuel_mass: 20100.0,
    burn_time: 115.0, // Two-stage solid, simplified to single boost
    area: 1.23, // 1.25m diameter
    isp_sea_level: 245.0,
    isp_vacuum: 290.0,
    pitch_start_time: 10.0,
    pitch_turn_rate: 0.7,
    boost_loft_factor: 1.1,
};

pub const HAJ_QASEM_SPECS: MissileSpecs = MissileSpecs {
    name: "Haj Qasem",
    dry_mass: 1000.0,
    fuel_mass: 5500.0,
    burn_time: 85.0,
    area: 0.5, 
    isp_sea_level: 240.0,
    isp_vacuum: 285.0,
    pitch_start_time: 8.0,
    pitch_turn_rate: 0.6,
    boost_loft_factor: 1.05,
};

pub const FATTAH_1_SPECS: MissileSpecs = MissileSpecs {
    name: "Fattah-1 (Boost Phase)",
    dry_mass: 2500.0,
    fuel_mass: 9000.0,
    burn_time: 105.0,
    area: 0.8,
    isp_sea_level: 250.0,
    isp_vacuum: 300.0,
    pitch_start_time: 12.0,
    pitch_turn_rate: 0.75,
    boost_loft_factor: 1.1,
};

// Global Models (US, France, India, North Korea)

pub const MINUTEMAN_III_SPECS: MissileSpecs = MissileSpecs {
    name: "LGM-30G Minuteman III (USA)",
    dry_mass: 3500.0,
    fuel_mass: 32000.0,
    burn_time: 180.0,
    area: 2.2, 
    isp_sea_level: 282.0,
    isp_vacuum: 305.0,
    pitch_start_time: 10.0,
    pitch_turn_rate: 0.85,
    boost_loft_factor: 1.25,
};

pub const TRIDENT_II_SPECS: MissileSpecs = MissileSpecs {
    name: "UGM-133 Trident II (USA/UK)",
    dry_mass: 5000.0,
    fuel_mass: 54000.0,
    burn_time: 160.0,
    area: 3.5,
    isp_sea_level: 280.0,
    isp_vacuum: 300.0,
    pitch_start_time: 8.0,
    pitch_turn_rate: 0.9,
    boost_loft_factor: 1.3,
};

pub const M51_SPECS: MissileSpecs = MissileSpecs {
    name: "M51 SLBM (France)",
    dry_mass: 4500.0,
    fuel_mass: 47500.0,
    burn_time: 170.0,
    area: 4.1,
    isp_sea_level: 280.0,
    isp_vacuum: 300.0,
    pitch_start_time: 10.0,
    pitch_turn_rate: 0.85,
    boost_loft_factor: 1.2,
};

pub const AGNI_V_SPECS: MissileSpecs = MissileSpecs {
    name: "Agni-V (India)",
    dry_mass: 5000.0,
    fuel_mass: 45000.0,
    burn_time: 190.0,
    area: 3.1,
    isp_sea_level: 275.0,
    isp_vacuum: 300.0,
    pitch_start_time: 15.0,
    pitch_turn_rate: 0.8,
    boost_loft_factor: 1.15,
};

pub const HWASONG_18_SPECS: MissileSpecs = MissileSpecs {
    name: "Hwasong-18 (North Korea)",
    dry_mass: 6000.0,
    fuel_mass: 50000.0,
    burn_time: 210.0,
    area: 3.5,
    isp_sea_level: 275.0,
    isp_vacuum: 305.0,
    pitch_start_time: 12.0,
    pitch_turn_rate: 0.75,
    boost_loft_factor: 1.1,
};

pub const DF_41_SPECS: MissileSpecs = MissileSpecs {
    name: "DF-41 (China)",
    dry_mass: 8000.0,
    fuel_mass: 72000.0,
    burn_time: 200.0,
    area: 3.14, // 2.0m diameter
    isp_sea_level: 285.0,
    isp_vacuum: 315.0,
    pitch_start_time: 15.0,
    pitch_turn_rate: 0.85,
    boost_loft_factor: 1.25,
};

pub const DF_17_SPECS: MissileSpecs = MissileSpecs {
    name: "DF-17 (China)",
    dry_mass: 4000.0,
    fuel_mass: 11000.0,
    burn_time: 110.0,
    area: 0.78, 
    isp_sea_level: 260.0,
    isp_vacuum: 300.0,
    pitch_start_time: 10.0,
    pitch_turn_rate: 0.8,
    boost_loft_factor: 1.1,
};

pub fn get_missile_registry() -> Vec<MissileSpecs> {
    if let Ok(json_str) = std::fs::read_to_string("assets/missiles.json") {
        let static_str: &'static str = Box::leak(json_str.into_boxed_str());
        if let Ok(registry) = serde_json::from_str(static_str) {
            return registry;
        } else {
            println!("⚠️ Failed to parse assets/missiles.json, falling back to internal registry.");
        }
    } else {
        println!("⚠️ Could not read assets/missiles.json, falling back to internal registry.");
    }

    vec![
        SCUD_C_SPECS,
        MRBM_SPECS,
        GHADR_SPECS,
        KHORRAMSHAHR_4_SPECS,
        SEJJIL_2_SPECS,
        HAJ_QASEM_SPECS,
        FATTAH_1_SPECS,
        MINUTEMAN_III_SPECS,
        TRIDENT_II_SPECS,
        M51_SPECS,
        AGNI_V_SPECS,
        HWASONG_18_SPECS,
        DF_41_SPECS,
        DF_17_SPECS,
    ]
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
        } else if altitude > 120000.0 || velocity.dot(position) >= 0.0 {
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

pub fn get_isa_properties(altitude: f64) -> (f64, f64, f64) {
    let (t, p) = if altitude < 11000.0 {
        let t = T0 - 0.0065 * altitude;
        let p = P0 * (t / T0).powf(G0 / (0.0065 * R_AIR));
        (t, p)
    } else if altitude < 20000.0 {
        let t = 216.65;
        let p_11 = P0 * (216.65 / T0).powf(G0 / (0.0065 * R_AIR));
        let p = p_11 * f64::exp(-G0 * (altitude - 11000.0) / (R_AIR * t));
        (t, p)
    } else if altitude < 32000.0 {
        let t = 216.65 + 0.001 * (altitude - 20000.0);
        let p_20 = 22632.0;
        let p = p_20 * (t / 216.65).powf(-G0 / (0.001 * R_AIR));
        (t, p)
    } else {
        let t = 228.65;
        let rho = 1.225 * f64::exp(-altitude / 8500.0);
        let p = rho * R_AIR * t;
        return (rho, (GAMMA * R_AIR * t).sqrt(), p);
    };

    let rho = p / (R_AIR * t);
    let sound_speed = (GAMMA * R_AIR * t).sqrt();
    (rho, sound_speed, p)
}

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
