//! Physics kernels used by trajectory integration: Earth shape, gravity, atmosphere, aero, propulsion.
//!
//! # Integration frame
//!
//! State is integrated in **game ECEF**: origin at Earth center, **+Y = North Pole**, prime meridian
//! toward **+X** (lon 0°). The equations of motion include **Coriolis** and **centrifugal** terms
//! (`-2Ω×v` and `−Ω×(Ω×r)`) in the rotating frame, with **Ω** along +Y (`EARTH_OMEGA`).
//! Gravitational acceleration is the gradient of the **gravitational** potential (not the centrifugal
//! potential); see `gravity_acceleration_ecef`.
//!
//! # Gravity
//!
//! `gravity_acceleration_ecef`: Newtonian central field plus zonal harmonics **J₂–J₄** (EGM96
//! coefficients, equatorial radius `WGS84_A`). Tesseral terms and third-body effects are omitted.
//!
//! # Atmosphere
//!
//! `get_isa_properties`: **US Standard Atmosphere 1976** from 0–86 km (geometric altitude), then a
//! thermosphere extension (interpolated ρ and T). Not NRLMSISE; see `docs/PHYSICS_MODEL.md` in the repo.
//!
//! # Aerodynamics
//!
//! `get_mach_drag`: tabulated **C_d(Mach)**; drag force ½ρv²C_dA opposes velocity in the vehicle layer.
//!
//! # Propulsion
//!
//! `mass_flow_kg_s` and `rocket_thrust_newton`: solid-motor-like **ṁ(τ)** and a linear pressure-thrust
//! model from Isp vacuum / sea level.
//!
//! # Integration (DOPRI5)
//!
//! [`dopri5_step_7`] / [`dopri5_step_6`] implement one step of the **Dormand–Prince 5(4)** pair
//! (order-5 update). [`inertial_accelerations`] packages gravity + Coriolis + centrifugal for the RHS.

mod aerodynamics;
mod atmosphere;
mod constants;
mod earth;
mod gravity;
mod integration;
mod propulsion;

pub use aerodynamics::get_mach_drag;
pub use atmosphere::get_isa_properties;
pub use constants::*;
pub use earth::{
    ecef_to_geodetic_deg, ecef_to_geodetic_deg_h, ecef_to_geodetic_rad_h, ellipsoid_height_m,
    geodetic_to_ecef, EARTH_OMEGA,
};
pub use gravity::gravity_acceleration_ecef;
pub use integration::{dopri5_step_6, dopri5_step_7, inertial_accelerations};
pub use propulsion::{mass_flow_kg_s, rocket_thrust_newton};
