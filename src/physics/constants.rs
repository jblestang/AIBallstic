//! Scalar physical constants shared across gravity, atmosphere, and propulsion.

// --- Earth shape & visualization ---
/// Mean spherical Earth radius (m). Used for visualization, zoom, and approximate great-circle distances.
/// Geodetic conversions and ellipsoid height use WGS84 `WGS84_A` / `WGS84_B`.
pub const EARTH_RADIUS: f64 = 6_371_000.0;

// --- WGS 84 ellipsoid ---
/// Semi-major axis (m).
pub const WGS84_A: f64 = 6_378_137.0;
/// Inverse flattening.
pub const WGS84_INV_F: f64 = 298.257223563;
/// Flattening f = 1 / inv_f.
pub const WGS84_F: f64 = 1.0 / WGS84_INV_F;
/// Semi-minor axis (m).
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F);
/// First eccentricity squared e² = (a² − b²) / a².
pub const WGS84_E2: f64 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_A * WGS84_A);
/// Second eccentricity squared e'² = (a² − b²) / b².
pub const WGS84_EP2: f64 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B);

// --- Dry air (ISA / thermosphere ideal gas) ---
/// Specific gas constant (J/(kg·K)).
pub const R_AIR: f64 = 287.05;
/// Heat capacity ratio (adiabatic index).
pub const GAMMA: f64 = 1.4;
/// Standard sea level temperature (K).
pub const T0: f64 = 288.15;
/// Standard sea level pressure (Pa).
pub const P0: f64 = 101_325.0;
/// Standard gravity (m/s²); used for Isp → effective exhaust velocity.
pub const G0: f64 = 9.80665;

// --- Gravity field ---
/// Universal gravitational constant (m³·kg⁻¹·s⁻²).
pub const G: f64 = 6.674_30e-11;
/// Earth mass (kg).
pub const EARTH_MASS: f64 = 5.972e24;
/// Standard gravitational parameter μ = G·M⊕ (m³/s²).
pub const GRAVITY_CONSTANT: f64 = G * EARTH_MASS;
/// J₂ zonal harmonic (dimensionless). EGM96 / WGS84; uses equatorial radius `WGS84_A` in formulas.
pub const EARTH_J2: f64 = 1.082_626_683_5e-3;
/// J₃ zonal harmonic (dimensionless). EGM96.
pub const EARTH_J3: f64 = -2.532_153_066e-6;
/// J₄ zonal harmonic (dimensionless). EGM96.
pub const EARTH_J4: f64 = -1.619_898_761e-6;
