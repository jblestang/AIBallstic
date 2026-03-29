//! Shared simulation code for the Bevy app and native CLI tools (e.g. `fuel_sweep`).
//!
//! - [`physics`] — gravity, atmosphere, Earth frame, aerodynamic and propulsion **kernels**
//! - [`missiles`] — vehicle specs, boost guidance, `BallisticMissilePhysics`
//! - [`sim`] — headless integration matching the game integrator
//!
//! See repository file `docs/PHYSICS_MODEL.md` for the physical model overview.

pub mod physics;
pub mod missiles;
pub mod sim;
