mod missiles;

use bevy::prelude::*;
use bevy::window::{Window, WindowPlugin};
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use glam::{DVec3, Vec3};
use nalgebra::{SMatrix, SVector};
use std::f32::consts::PI;
use missiles::*;
use chrono::{Datelike, Timelike, Utc};

// Constants
// Real-world alignment is handled in missiles.rs constants

struct SafeDespawn(Entity);

impl bevy::ecs::system::Command for SafeDespawn {
    fn apply(self, world: &mut World) {
        if let Ok(e) = world.get_entity_mut(self.0) {
            e.despawn();
        }
    }
}

fn safe_despawn(commands: &mut Commands, entity: Entity) {
    commands.queue(SafeDespawn(entity));
}

#[derive(Component)]
pub struct Missile {
    pub position_ecef: DVec3,
    pub start_position_ecef: DVec3,
    pub velocity_ecef: DVec3,
    pub mass: f64,
    pub timer: f32,
    pub phase: FlightPhase,
    pub path: Vec<(Vec3, FlightPhase)>,
    pub model: Box<dyn PhysicsModel>,
}

#[derive(Component)]
struct Earth;

#[derive(Component)]
struct Sun;

#[derive(Resource)]
struct SimulationSettings {
    time_scale: f32,
    rotation_paused: bool,
    zoom_distance: f32,
    theta: f32, // Horizontal angle
    phi: f32,   // Vertical angle
    coriolis_enabled: bool,
    centrifugal_enabled: bool,
    show_radar_coverage: bool,
    texture_lon_offset: f32,
}

#[derive(Resource)]
pub struct ActiveMissileSpecs(pub MissileSpecs);

#[derive(Resource)]
pub struct MissileRegistry(pub Vec<MissileSpecs>);

// Radar Tracking Resources
#[derive(Resource, Default)]
struct TrackedMissileState {
    pub position_ecef: Option<DVec3>,
    pub velocity_ecef: Option<DVec3>,
    // EKF 6D State [x, y, z, vx, vy, vz] and Covariance Matrix
    pub ekf_state: Option<SVector<f64, 6>>,
    pub ekf_covariance: Option<SMatrix<f64, 6, 6>>,
}

#[derive(Resource, Default)]
struct ImpactPrediction {
    pub coordinates_ecef: Vec<DVec3>,
    pub centroid_ecef: Option<DVec3>,
}

#[derive(Resource, Default)]
struct ImpactErrorHistory {
    pub data: Vec<[f64; 2]>,
}

#[derive(Resource, Default)]
struct MissileFlightHistory {
    pub velocity: Vec<[f64; 2]>,
    pub mach: Vec<[f64; 2]>,
    pub altitude: Vec<[f64; 2]>,
}

#[derive(Component)]
struct RadarStation {
    pub position_ecef: DVec3,
    pub range: f64,
    pub scan_timer: f32,
}

// ABM Interception Structures
#[derive(Component)]
struct DefendedZone {
    pub position_ecef: DVec3,
    pub radius: f64, // e.g., 500km Defense Radius
}

#[derive(Component)]
struct ABMInterceptor {
    pub target_entity: Entity,
    pub position_ecef: DVec3,
    pub velocity_ecef: DVec3,
    pub navigation_gain: f64, // PN gain typically 3.0 to 5.0
    pub max_g_load: f64, // Maximum lateral acceleration
    pub kill_radius: f64,
}

#[derive(Clone, Copy, Debug)]
struct SpawnABMEvent {
    pub target_entity: Entity,
    pub battery_pos: DVec3,
}

#[derive(Resource, Default)]
struct ABMLaunchQueue(Vec<SpawnABMEvent>);
#[derive(Component)]
struct Explosion {
    pub timer: f32,
    pub max_time: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ReentryBodyType {
    Warhead,
    Decoy,
}

#[derive(Component)]
pub struct ReentryBody {
    pub body_type: ReentryBodyType,
    pub position_ecef: DVec3,
    pub velocity_ecef: DVec3,
    pub mass: f64,
    pub area: f64,
    pub path: Vec<(Vec3, ReentryBodyType)>,
    pub phase: FlightPhase,
    pub parent_missile: Option<Entity>,
}

#[derive(Resource, Default)]
struct MirvDeploymentTracker {
    deployed_missiles: Vec<Entity>,
}


fn main() {
    let args: Vec<String> = std::env::args().collect();
    let registry = get_missile_registry();

    if args.iter().any(|arg| arg == "--list" || arg == "-l") {
        println!("\n🚀 AIBallistic Missile Registry");
        println!("===============================");
        for specs in &registry {
            println!("- {}", specs.name);
        }
        println!("\nUsage: cargo run --release -- --missile \"<name>\"\n");
        return;
    }

    let mut selected_specs = registry.first().cloned().expect("Missile registry is empty!");
    if let Some(pos) = args.iter().position(|arg| arg == "--missile" || arg == "-m") {
        if let Some(target_name) = args.get(pos + 1) {
            if let Some(specs) = registry.iter().find(|s| s.name.to_lowercase().contains(&target_name.to_lowercase())) {
                selected_specs = specs.clone();
                println!("✅ Selected missile: {}", selected_specs.name);
            } else {
                println!("❌ Missile '{}' not found. Available:", target_name);
                for specs in &registry {
                    println!("- {}", specs.name);
                }
                return;
            }
        }
    }

    #[cfg(target_arch = "wasm32")]
    let window_plugin = WindowPlugin {
        primary_window: Some(Window {
            canvas: Some("#bevy-canvas".into()),
            fit_canvas_to_parent: true,
            ..default()
        }),
        ..default()
    };
    #[cfg(not(target_arch = "wasm32"))]
    let window_plugin = WindowPlugin::default();

    App::new()
        .add_plugins(DefaultPlugins.set(ImagePlugin::default_nearest()).set(window_plugin))
        .add_plugins(EguiPlugin::default())
        .insert_resource(MissileRegistry(registry))
        .insert_resource(ActiveMissileSpecs(selected_specs))
        .insert_resource(SimulationSettings { 
            time_scale: 10.0, 
            rotation_paused: false,
            zoom_distance: EARTH_RADIUS as f32 * 2.5,
            theta: 0.0,
            phi: 0.5, // Slight tilt
            coriolis_enabled: true,
            centrifugal_enabled: true,
            show_radar_coverage: true,
            texture_lon_offset: 0.0, // Base UV mathematically aligns perfectly
        })
        .init_resource::<TrackedMissileState>()
        .init_resource::<ImpactPrediction>()
        .init_resource::<ImpactErrorHistory>()
        .init_resource::<MissileFlightHistory>()
        .init_resource::<ABMLaunchQueue>()
        .init_resource::<MirvDeploymentTracker>()
        .add_systems(Startup, setup)
        .add_systems(EguiPrimaryContextPass, egui_stats_system)
        .add_systems(Update, (
            physics_system, trajectory_system, camera_system, input_system, 
            solar_lighting_system, radar_scan_system, impact_prediction_system, earth_alignment_system,
            abm_c2_system, spawn_abm_system, abm_guidance_system, abm_kill_system, explosion_system,
            mirv_deployment_system, reentry_body_physics_system,
        ))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    active_specs: Res<ActiveMissileSpecs>,
) {
    // Earth (North Pole at +Y, Prime Meridian at +X)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(EARTH_RADIUS as f32).mesh().uv(128, 64))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color_texture: Some(asset_server.load("earth_day_texture.jpg")),
            emissive: bevy::color::Color::srgb(4.0, 3.5, 2.5).into(),
            emissive_texture: Some(asset_server.load("earth_night_texture.jpg")),
            unlit: false,
            ..default()
        })),
        Earth,
        Transform::from_rotation(Quat::from_mat3(&bevy::math::Mat3::from_cols(
            Vec3::new(-1.0, 0.0, 0.0), // X col
            Vec3::new(0.0, 0.0, 1.0),  // Y col
            Vec3::new(0.0, 1.0, 0.0),  // Z col
        ))),
    ));

    // Light (Sun)
    commands.spawn((
        DirectionalLight {
            illuminance: 50000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0 * EARTH_RADIUS as f32, 0.0, 2.0 * EARTH_RADIUS as f32).looking_at(Vec3::ZERO, Vec3::Y),
        Sun
    ));



    // Radar Station placed 1000 km south of Moscow (bearing south).
    // (Used for tracking the Tehran->Moscow flight in this scenario.)
    let (radar_lat, radar_lon) = destination_point(55.7558, 37.6173, 180.0, 1_000_000.0);
    commands.spawn(RadarStation {
        position_ecef: geodetic_to_ecef(radar_lat, radar_lon, 100.0),
        range: 5_000_000.0, // 5000 km
        scan_timer: 0.0,
    });

    // Defended Zone (ABM site) placed 500 km south of Moscow, then 100 km to the south-east, then 100 km east, then 100 km south-east.
    let (adm_lat0, adm_lon0) = destination_point(55.7558, 37.6173, 180.0, 500_000.0);
    let (adm_lat, adm_lon) = destination_point(adm_lat0, adm_lon0, 135.0, 100_000.0);

    commands.spawn(DefendedZone {
        position_ecef: geodetic_to_ecef(adm_lat, adm_lon, 0.0),
        radius: 300_000.0, // 300 km point-defense radius
    });

    // Additional Defended Zone: 200 km east of Moscow, then +100 km south-east, then +100 km south-east again
    let (moscow_abm_lat0, moscow_abm_lon0) = destination_point(MOSCOW_LAT, MOSCOW_LON, 90.0, 200_000.0);
    let (moscow_abm_lat1, moscow_abm_lon1) = destination_point(moscow_abm_lat0, moscow_abm_lon0, 135.0, 100_000.0);
    let (moscow_abm_lat, moscow_abm_lon) = destination_point(moscow_abm_lat1, moscow_abm_lon1, 135.0, 100_000.0);
    commands.spawn(DefendedZone {
        position_ecef: geodetic_to_ecef(moscow_abm_lat, moscow_abm_lon, 0.0),
        radius: 300_000.0, // 300 km point-defense radius
    });

    // Launch from TEHRAN, IR
    // Geodetic: 35.6892° N, 51.3890° E
    let tehran_ecef = geodetic_to_ecef(35.6892, 51.3890, 10.0);
    let specs = active_specs.0.clone();
    let initial_mass = specs.total_mass();
    
    commands.spawn(Missile {
        position_ecef: tehran_ecef,
        start_position_ecef: tehran_ecef,
        velocity_ecef: DVec3::ZERO,
        mass: initial_mass,
        timer: 0.0,
        phase: FlightPhase::Boost,
        path: Vec::new(),
        model: Box::new(BallisticMissilePhysics::new(specs)),
    });

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(EARTH_RADIUS as f32 * 2.5, EARTH_RADIUS as f32 * 1.5, EARTH_RADIUS as f32 * 2.5)
            .looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn physics_system(
    time: Res<Time>,
    settings: Res<SimulationSettings>,
    mut query: Query<&mut Missile>,
    mut flight_history: ResMut<MissileFlightHistory>,
) {
    let dt = time.delta_secs() * settings.time_scale;

    for mut missile in query.iter_mut() {
        if missile.phase == FlightPhase::Landed { continue; }

        let r = missile.position_ecef.length();
        if r < EARTH_RADIUS && missile.timer > 1.0 {
             missile.velocity_ecef = DVec3::ZERO;
             missile.phase = FlightPhase::Landed;
             continue;
        }

        // --- 1. Global Field Accelerations ---
        // Gravity Equation: a = - (G * M) / r^2 * (r_hat)
        // [Reference: https://en.wikipedia.org/wiki/Newton%27s_law_of_universal_gravitation]
        let gravity_dir = -missile.position_ecef.normalize();
        let gravity_accel = gravity_dir * (GRAVITY_CONSTANT / (r * r));

        let v_ecef = missile.velocity_ecef;
        let r_ecef = missile.position_ecef;
        
        // Coriolis Effect: Apparent deflection due to rotating reference frame.
        // Equation: a_coriolis = -2 * (Omega x v)
        // [Reference: https://en.wikipedia.org/wiki/Coriolis_force]
        let coriolis_accel = if settings.coriolis_enabled {
            -2.0 * EARTH_OMEGA.cross(v_ecef)
        } else {
            DVec3::ZERO
        };
        
        // Centrifugal Force: Outward apparent force in a rotating reference frame.
        // Equation: a_centrifugal = - Omega x (Omega x r)
        // Causes the Earth's equatorial bulge and reduces apparent gravity at the equator.
        // [Reference: https://en.wikipedia.org/wiki/Centrifugal_force]
        let centrifugal_accel = if settings.centrifugal_enabled {
            -EARTH_OMEGA.cross(EARTH_OMEGA.cross(r_ecef))
        } else {
            DVec3::ZERO
        };

        // --- 2. Missile-Specific Accelerations ---
        // Pluggable Model Specific Acceleration (Thrust, Aerodynamic Drag, etc.)
        // These are calculated in the local frame of the missile based on its state.
        let (model_accel, new_phase, mass_delta) = missile.model.compute_acceleration(
            missile.position_ecef, 
            missile.velocity_ecef, 
            missile.mass, 
            missile.timer,
            dt
        );

        missile.phase = new_phase;
        missile.mass += mass_delta;
        missile.timer += dt;

        // --- 3. Numerical Integration ---
        // Using Semi-Implicit Euler integration for trajectory propagation.
        // This is a symplectic integrator, meaning it better preserves energy 
        // over time in orbital/ballistic mechanics compared to explicit Euler.
        // [Reference: https://en.wikipedia.org/wiki/Semi-implicit_Euler_method]
        let total_accel = gravity_accel + coriolis_accel + centrifugal_accel + model_accel;
        
        // v_{n+1} = v_n + a_n * dt
        missile.velocity_ecef += total_accel * dt as f64;
        let vel = missile.velocity_ecef;
        
        // x_{n+1} = x_n + v_{n+1} * dt
        missile.position_ecef += vel * dt as f64;
        
        // --- 4. Trajectory Tracking ---
        const MAX_PATH_LEN: usize = 3000;
        const MAX_HISTORY_LEN: usize = 2500;
        if missile.timer % 1.0 < dt {
            let pos = missile.position_ecef;
            let phase = missile.phase;
            missile.path.push((pos.as_vec3(), phase));
            let path_len = missile.path.len();
            if path_len > MAX_PATH_LEN {
                missile.path.drain(0..path_len - MAX_PATH_LEN);
            }

            let altitude_m = (pos.length() - EARTH_RADIUS).max(0.0);
            let alt_km = altitude_m / 1000.0;
            let speed_ms = vel.length();
            let (_, sound_speed, _) = get_isa_properties(altitude_m);
            let mach = speed_ms / sound_speed;
            let t = missile.timer as f64;
            flight_history.altitude.push([t, alt_km]);
            flight_history.velocity.push([t, speed_ms]);
            flight_history.mach.push([t, mach]);
            let hist_len = flight_history.velocity.len();
            if hist_len > MAX_HISTORY_LEN {
                let n = hist_len - MAX_HISTORY_LEN;
                flight_history.altitude.drain(0..n);
                flight_history.velocity.drain(0..n);
                flight_history.mach.drain(0..n);
            }
        }
    }
}

// ----------------------------------------------------------------------------
// Radar & Impact Prediction Systems
// ----------------------------------------------------------------------------

#[allow(non_snake_case)]
fn radar_scan_system(
    time: Res<Time>,
    mut radar_query: Query<&mut RadarStation>,
    missile_query: Query<&Missile>,
    mut tracked_state: ResMut<TrackedMissileState>,
) {
    let dt = time.delta_secs();
    
    for mut radar in radar_query.iter_mut() {
        radar.scan_timer += dt;
        
        // 1Hz refresh rate
        if radar.scan_timer >= 1.0 {
            let scan_dt = radar.scan_timer;
            radar.scan_timer = 0.0;
            
            for missile in missile_query.iter() {
                // If landed, stop tracking
                if missile.phase == FlightPhase::Landed {
                    continue;
                }
                
                let dist = radar.position_ecef.distance(missile.position_ecef);
                
                // Simple Line of Sight Check (is it above the horizon from radar's perspective?)
                let radar_norm = radar.position_ecef.normalize();
                let vec_to_missile = (missile.position_ecef - radar.position_ecef).normalize();
                let is_above_horizon = radar_norm.dot(vec_to_missile) > 0.0;
                
                if dist <= radar.range && is_above_horizon {
                    // Radar has acquired the target! Update tracked state with NOISE.
                    use rand::Rng;
                    let mut rng = rand::thread_rng();

                    // Raw noisy measurement
                    let pos_noise = DVec3::new(
                        rng.gen_range(-1000.0..1000.0), rng.gen_range(-1000.0..1000.0), rng.gen_range(-1000.0..1000.0)
                    );
                    let vel_noise = DVec3::new(
                        rng.gen_range(-50.0..50.0), rng.gen_range(-50.0..50.0), rng.gen_range(-50.0..50.0)
                    );
                    
                    let measured_pos = missile.position_ecef + pos_noise;
                    let measured_vel = missile.velocity_ecef + vel_noise;
                    
                    let z = SVector::<f64, 6>::new(
                        measured_pos.x, measured_pos.y, measured_pos.z,
                        measured_vel.x, measured_vel.y, measured_vel.z
                    );

                    // Measurement Noise Covariance R (Pos variance ~ (1000^2)/12 = 83333, Vel ~ (100^2)/12 = 833)
                    // Let's use simpler explicit bounds
                    let mut R = SMatrix::<f64, 6, 6>::zeros();
                    R[(0,0)] = 85_000.0; R[(1,1)] = 85_000.0; R[(2,2)] = 85_000.0;
                    R[(3,3)] = 850.0;    R[(4,4)] = 850.0;    R[(5,5)] = 850.0;

                    if tracked_state.ekf_state.is_none() {
                        // First acquisition: Initialize EKF directly to measurement
                        tracked_state.ekf_state = Some(z);
                        tracked_state.ekf_covariance = Some(R); // Initial uncertainty equals measurement noise
                    } else {
                        // EKF Predict & Update Step
                        let mut x = tracked_state.ekf_state.unwrap();
                        let mut P = tracked_state.ekf_covariance.unwrap();
                        
                        // 1. Predict Step (Propagate state forward by scan_dt)
                        let mut x_pred = x;
                        let mut pos = DVec3::new(x[0], x[1], x[2]);
                        let mut vel = DVec3::new(x[3], x[4], x[5]);
                        
                        // Break the integration into smaller steps for stability
                        let integration_steps = 10;
                        let step_dt = scan_dt / (integration_steps as f32);
                        
                        for _ in 0..integration_steps {
                            let r = pos.length();
                            let altitude = (r - EARTH_RADIUS).max(0.0);
                            let gravity = -pos.normalize() * (GRAVITY_CONSTANT / (r * r));
                            
                            let (rho, sound, _) = get_isa_properties(altitude);
                            let mach = vel.length() / sound;
                            let cd = get_mach_drag(mach);
                            let drag_force = -0.5 * rho * vel.length_squared() * cd * 0.5; // est area 0.5
                            let drag_accel = if vel.length() > 1e-3 { (vel.normalize() * drag_force) / 500.0 } else { DVec3::ZERO }; // est mass 500
                            
                            let total_accel = gravity + drag_accel;
                            
                            vel += total_accel * step_dt as f64;
                            pos += vel * step_dt as f64;
                        }
                        
                        // New predicted state
                        x_pred[0] = pos.x; x_pred[1] = pos.y; x_pred[2] = pos.z;
                        x_pred[3] = vel.x; x_pred[4] = vel.y; x_pred[5] = vel.z;

                        // Process Noise Covariance Q (uncertainty in our physics model vs reality)
                        let mut q = SMatrix::<f64, 6, 6>::zeros();
                        // Drastically increase Q because our physics model assumes NO thrust (ballistic re-entry).
                        // When the missile is boosting, it accelerates greatly, and a small Q causes the filter to diverge!
                        q[(0,0)] = 50_000.0; q[(1,1)] = 50_000.0; q[(2,2)] = 50_000.0;
                        q[(3,3)] = 5_000.0;  q[(4,4)] = 5_000.0;  q[(5,5)] = 5_000.0;
                        
                        // Numerical Jacobian F for State Transition mapping
                        let mut f = SMatrix::<f64, 6, 6>::identity();
                        // Position derives from Velocity over scan_dt
                        f[(0,3)] = scan_dt as f64; f[(1,4)] = scan_dt as f64; f[(2,5)] = scan_dt as f64;
                        // Approximate the rest to save deep partial derivatives
                        
                        let p_pred = f * P * f.transpose() + q;

                        // 2. Update Step
                        let h = SMatrix::<f64, 6, 6>::identity(); // We measure all states directly
                        let s = h * p_pred * h.transpose() + R;
                        let k = p_pred * h.transpose() * s.try_inverse().unwrap_or(SMatrix::identity()); // Kalman Gain
                        
                        let y = z - (h * x_pred); // Innovation
                        x = x_pred + k * y;
                        let i = SMatrix::<f64, 6, 6>::identity();
                        P = (i - k * h) * p_pred;

                        tracked_state.ekf_state = Some(x);
                        tracked_state.ekf_covariance = Some(P);
                    }
                    
                    // Expose the clean filtered states for prediction engine
                    let final_x = tracked_state.ekf_state.unwrap();
                    tracked_state.position_ecef = Some(DVec3::new(final_x[0], final_x[1], final_x[2]));
                    tracked_state.velocity_ecef = Some(DVec3::new(final_x[3], final_x[4], final_x[5]));
                }
            }
        }
    }
}

fn compute_ideal_delta_v(specs: &MissileSpecs) -> f64 {
    if specs.is_multistage() {
        let mut total_dv = 0.0;
        let mut current_mass = specs.total_mass();
        for stage in &specs.stages {
            let m_after_burn = current_mass - stage.fuel_mass;
            if m_after_burn > 0.0 && current_mass > 0.0 {
                total_dv += stage.isp_vacuum * G0 * (current_mass / m_after_burn).ln();
            }
            current_mass = m_after_burn - stage.dry_mass;
        }
        total_dv
    } else {
        let m0 = specs.dry_mass + specs.fuel_mass;
        let mf = specs.dry_mass;
        if mf > 0.0 && m0 > 0.0 {
            specs.isp_vacuum * G0 * (m0 / mf).ln()
        } else {
            0.0
        }
    }
}

fn compute_ballistic_impact(
    start_pos: DVec3,
    start_vel: DVec3,
    settings: &SimulationSettings,
    est_mass: f64,
    est_area: f64,
) -> DVec3 {
    let dt: f32 = 1.0;
    let mut pos = start_pos;
    let mut vel = start_vel;
    let mut predicted_alt = (pos.length() - EARTH_RADIUS).max(0.0);
    let mut iterations = 0;
    #[cfg(target_arch = "wasm32")]
    const MAX_ITER: usize = 2500;
    #[cfg(not(target_arch = "wasm32"))]
    const MAX_ITER: usize = 5000;

    while predicted_alt > 0.0 && iterations < MAX_ITER {
        iterations += 1;
        let r = pos.length();
        let altitude = (r - EARTH_RADIUS).max(0.0);

        let gravity_dir = -pos.normalize();
        let gravity_accel = gravity_dir * (GRAVITY_CONSTANT / (r * r));

        let coriolis_accel = if settings.coriolis_enabled {
            -2.0 * EARTH_OMEGA.cross(vel)
        } else {
            DVec3::ZERO
        };

        let centrifugal_accel = if settings.centrifugal_enabled {
            -EARTH_OMEGA.cross(EARTH_OMEGA.cross(pos))
        } else {
            DVec3::ZERO
        };

        let (rho, sound_speed, _) = get_isa_properties(altitude);
        let speed = vel.length();
        let mach = speed / sound_speed;
        let cd = get_mach_drag(mach);

        let drag_force = -0.5 * rho * (speed * speed) * cd * est_area;
        let drag_accel = if speed > 1e-3 {
            (vel.normalize() * drag_force) / est_mass
        } else {
            DVec3::ZERO
        };

        let total_accel = gravity_accel + coriolis_accel + centrifugal_accel + drag_accel;

        vel += total_accel * dt as f64;
        pos += vel * dt as f64;

        predicted_alt = (pos.length() - EARTH_RADIUS).max(0.0);
    }

    pos
}

fn impact_prediction_system(
    tracked_state: Res<TrackedMissileState>,
    mut prediction: ResMut<ImpactPrediction>,
    settings: Res<SimulationSettings>,
    missile_query: Query<&Missile>,
    mut error_history: ResMut<ImpactErrorHistory>,
) {
    if !tracked_state.is_changed() || tracked_state.ekf_state.is_none() {
        return;
    }
    
    let ekf = tracked_state.ekf_state.unwrap();
    let base_pos = DVec3::new(ekf[0], ekf[1], ekf[2]);
    let base_vel = DVec3::new(ekf[3], ekf[4], ekf[5]);
    
    prediction.coordinates_ecef.clear();

    // Fewer Monte Carlo runs on WASM to avoid blocking the main thread
    #[cfg(target_arch = "wasm32")]
    let n_simulations = 25usize;
    #[cfg(not(target_arch = "wasm32"))]
    let n_simulations = 100;
    use rand::Rng;
    use rand_distr::{Distribution, StandardNormal};
    let mut rng = rand::thread_rng();

    // Use EKF covariance to generate properly correlated state perturbations.
    // Cholesky decomposition: P = L * L^T, then sample x_perturbed = x_ekf + L * z
    // where z ~ N(0, I). This ensures the Monte Carlo scatter reflects the actual
    // tracking uncertainty from radar noise, not arbitrary hardcoded values.
    let cholesky_l = tracked_state.ekf_covariance
        .and_then(|p| p.cholesky())
        .map(|c| c.unpack());

    for _ in 0..n_simulations {
        let (perturbed_pos, perturbed_vel) = if let Some(l) = &cholesky_l {
            let z = SVector::<f64, 6>::from_fn(|_, _| StandardNormal.sample(&mut rng));
            let dx = l * z;
            (
                base_pos + DVec3::new(dx[0], dx[1], dx[2]),
                base_vel + DVec3::new(dx[3], dx[4], dx[5]),
            )
        } else {
            // Fallback if covariance is not available or not positive-definite
            (
                base_pos + DVec3::new(
                    rng.gen_range(-500.0..500.0),
                    rng.gen_range(-500.0..500.0),
                    rng.gen_range(-500.0..500.0),
                ),
                base_vel + DVec3::new(
                    rng.gen_range(-10.0..10.0),
                    rng.gen_range(-10.0..10.0),
                    rng.gen_range(-10.0..10.0),
                ),
            )
        };

        let est_mass = rng.gen_range(400.0..600.0);
        let est_area = rng.gen_range(0.4..0.6);
        
        let impact = compute_ballistic_impact(perturbed_pos, perturbed_vel, &settings, est_mass, est_area);
        prediction.coordinates_ecef.push(impact);
    }

    if !prediction.coordinates_ecef.is_empty() {
        let mut centroid = DVec3::ZERO;
        for p in &prediction.coordinates_ecef {
            centroid += *p;
        }
        centroid /= prediction.coordinates_ecef.len() as f64;
        prediction.centroid_ecef = Some(centroid);

        if let Some(missile) = missile_query.iter().next() {
            let true_impact = compute_ballistic_impact(
                missile.position_ecef,
                missile.velocity_ecef,
                &settings,
                500.0,
                0.5,
            );
            let error_km = centroid.distance(true_impact) / 1000.0;
            error_history.data.push([missile.timer as f64, error_km]);
            const MAX_ERROR_HISTORY: usize = 500;
            let len = error_history.data.len();
            if len > MAX_ERROR_HISTORY {
                error_history.data.drain(0..len - MAX_ERROR_HISTORY);
            }
        }
    }
}

// ----------------------------------------------------------------------------
// ABM Systems
// ----------------------------------------------------------------------------

fn abm_c2_system(
    mut launch_queue: ResMut<ABMLaunchQueue>,
    prediction: Res<ImpactPrediction>,
    defended_zones: Query<&DefendedZone>,
    missile_query: Query<(Entity, &Missile)>,
    reentry_query: Query<(Entity, &ReentryBody)>,
    interceptor_query: Query<&ABMInterceptor>,
) {
    if prediction.coordinates_ecef.is_empty() { return; }
    
    let mut centroid = DVec3::ZERO;
    for &pt in &prediction.coordinates_ecef {
        centroid += pt;
    }
    centroid /= prediction.coordinates_ecef.len() as f64;
    
    for zone in defended_zones.iter() {
        if centroid.distance(zone.position_ecef) > zone.radius { continue; }

        // Collect all threat entities (missiles + reentry bodies)
        let mut threats: Vec<(Entity, DVec3, bool)> = Vec::new();

        for (entity, missile) in missile_query.iter() {
            if missile.phase == FlightPhase::Landed { continue; }
            threats.push((entity, missile.position_ecef, true));
        }

        for (entity, body) in reentry_query.iter() {
            if body.phase == FlightPhase::Landed { continue; }
            // During reentry, decoys decelerate faster → discriminated
            // Only engage decoys if still in mid-course (Ballistic phase)
            let dominated_by_drag = body.phase == FlightPhase::ReEntry
                && body.body_type == ReentryBodyType::Decoy;
            if dominated_by_drag { continue; }
            threats.push((entity, body.position_ecef, false));
        }

        for (threat_entity, threat_pos, _is_missile) in &threats {
            let mut already_targeted = false;
            for int in interceptor_query.iter() {
                if int.target_entity == *threat_entity { already_targeted = true; break; }
            }
            if already_targeted { continue; }

            let dist_to_zone = threat_pos.distance(zone.position_ecef);
            if dist_to_zone < 500_000.0 {
                let battery_pos = zone.position_ecef + (zone.position_ecef.normalize() * 100.0);
                launch_queue.0.push(SpawnABMEvent {
                    target_entity: *threat_entity,
                    battery_pos,
                });
            }
        }
    }
}

fn spawn_abm_system(
    mut commands: Commands,
    mut launch_queue: ResMut<ABMLaunchQueue>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for event in launch_queue.0.drain(..) {
        println!("🚀 BATTERY LAUNCH: ABM Interceptor fired to destroy target {:?}", event.target_entity);
        
        commands.spawn((
            Mesh3d(meshes.add(Sphere::new(50000.0).mesh().uv(32, 16))), 
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: bevy::color::Color::srgb(0.0, 1.0, 0.0), // Green for Interceptors
                unlit: true,
                ..default()
            })),
            Transform::from_translation(event.battery_pos.as_vec3()),
            ABMInterceptor {
                target_entity: event.target_entity,
                position_ecef: event.battery_pos,
                velocity_ecef: event.battery_pos.normalize() * 500.0, // Booster fast launch speed
                navigation_gain: 4.0, 
                max_g_load: 50.0 * 9.81, 
                kill_radius: 50_000.0, 
            }
        ));
    }
}

fn abm_guidance_system(
    time: Res<Time>,
    settings: Res<SimulationSettings>,
    mut commands: Commands,
    mut interceptor_query: Query<(Entity, &mut ABMInterceptor, &mut Transform)>,
    missile_query: Query<(Entity, &Missile)>,
    reentry_query: Query<(Entity, &ReentryBody)>,
    tracked_state: Res<TrackedMissileState>,
) {
    let dt = time.delta_secs() * settings.time_scale;
    
    for (int_ent, mut abm, mut transform) in interceptor_query.iter_mut() {
        // Find target among missiles or reentry bodies
        let target_state: Option<(DVec3, DVec3, bool)> = 
            if let Some((_, m)) = missile_query.iter().find(|(e, _)| *e == abm.target_entity) {
                if m.phase == FlightPhase::Landed { None }
                else {
                    let tp = tracked_state.position_ecef.unwrap_or(m.position_ecef);
                    let tv = tracked_state.velocity_ecef.unwrap_or(m.velocity_ecef);
                    Some((tp, tv, false))
                }
            } else if let Some((_, b)) = reentry_query.iter().find(|(e, _)| *e == abm.target_entity) {
                if b.phase == FlightPhase::Landed { None }
                else { Some((b.position_ecef, b.velocity_ecef, false)) }
            } else {
                None
            };

        let Some((target_pos, target_vel, _)) = target_state else {
            safe_despawn(&mut commands, int_ent);
            continue;
        };

        let los = target_pos - abm.position_ecef;
        let los_dist = los.length();
        if los_dist < 1.0 { continue; } 
        let los_dir = los.normalize();
        
        let rel_vel = target_vel - abm.velocity_ecef;
        let closing_vel = -rel_vel.dot(los_dir);
        let los_rate_vector = los_dir.cross(rel_vel) / los_dist;
        
        let mut accel = DVec3::ZERO;
        if closing_vel > 0.0 {
            accel = abm.navigation_gain * closing_vel * los_rate_vector.cross(los_dir);
            if accel.length() > abm.max_g_load {
                accel = accel.normalize() * abm.max_g_load;
            }
        }
        
        let r = abm.position_ecef.length();
        let gravity = -abm.position_ecef.normalize() * (GRAVITY_CONSTANT / (r * r));
        accel += gravity;
        
        let speed = abm.velocity_ecef.length();
        if speed < 5000.0 {
            let thrust_dir = if speed > 1.0 { abm.velocity_ecef.normalize() } else { los_dir };
            accel += thrust_dir * (30.0 * 9.81); 
        }
        
        abm.velocity_ecef += accel * dt as f64;
        let vel_step = abm.velocity_ecef * dt as f64;
        abm.position_ecef += vel_step;
        transform.translation = abm.position_ecef.as_vec3();
    }
}

fn abm_kill_system(
    mut commands: Commands,
    interceptor_query: Query<(Entity, &ABMInterceptor)>,
    missile_query: Query<(Entity, &Missile)>,
    reentry_query: Query<(Entity, &ReentryBody)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (int_ent, abm) in interceptor_query.iter() {
        let (target_pos, target_ent) = 
            if let Ok((e, m)) = missile_query.get(abm.target_entity) {
                (m.position_ecef, Some(e))
            } else if let Ok((e, b)) = reentry_query.get(abm.target_entity) {
                (b.position_ecef, Some(e))
            } else {
                safe_despawn(&mut commands, int_ent);
                continue;
            };

        let Some(target_ent) = target_ent else { continue; };
        let dist = abm.position_ecef.distance(target_pos);
        if dist < abm.kill_radius {
            let alt = (target_pos.length() - EARTH_RADIUS) / 1000.0;
            println!("💥 INTERCEPT SUCCESSFUL! Threat neutralized at {:.1} km altitude.", alt);
            
            commands.spawn((
                Mesh3d(meshes.add(Sphere::new(100_000.0).mesh().uv(32, 16))), 
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: bevy::color::Color::srgb(1.0, 0.5, 0.0),
                    unlit: true,
                    ..default()
                })),
                Transform::from_translation(abm.position_ecef.as_vec3()),
                Explosion { timer: 0.0, max_time: 2.0 }
            ));
            
            safe_despawn(&mut commands, target_ent);
            safe_despawn(&mut commands, int_ent);
        }
    }
}

fn explosion_system(
    time: Res<Time>,
    mut commands: Commands,
    mut q: Query<(Entity, &mut Explosion, &mut Transform)>,
) {
    for (e, mut exp, mut transform) in q.iter_mut() {
        exp.timer += time.delta_secs();
        if exp.timer >= exp.max_time {
            safe_despawn(&mut commands, e);
        } else {
            let scale = 1.0 - (exp.timer / exp.max_time);
            transform.scale = Vec3::splat(scale.max(0.01));
        }
    }
}

// ----------------------------------------------------------------------------
// Visualization System
// ----------------------------------------------------------------------------

fn trajectory_system(
    mut gizmos: Gizmos,
    query: Query<&Missile>,
    reentry_query: Query<&ReentryBody>,
    radar_query: Query<&RadarStation>,
    defended_zones: Query<&DefendedZone>,
    interceptor_query: Query<&ABMInterceptor>,
    prediction: Res<ImpactPrediction>,
    settings: Res<SimulationSettings>,
) {
    // Target Markers
    // Tehran - RED
    let tehran = geodetic_to_ecef(35.6892, 51.3890, 0.0);
    gizmos.sphere(tehran.as_vec3(), 50000.0, bevy::color::palettes::css::RED);

    // Moscow - PURPLE
    let moscow = geodetic_to_ecef(MOSCOW_LAT, MOSCOW_LON, 0.0);
    gizmos.sphere(moscow.as_vec3(), 65000.0, bevy::color::palettes::css::PURPLE);

    // Radar Stations
    if settings.show_radar_coverage {
        for radar in radar_query.iter() {
            // Draw radar station
            gizmos.sphere(radar.position_ecef.as_vec3(), 75000.0, bevy::color::palettes::css::BLUE);
        }
    }

    // Defended Zones
    for zone in defended_zones.iter() {
        let p_vec = zone.position_ecef.as_vec3();
        let up = p_vec.normalize();
        
        // Draw the zone origin point
        gizmos.sphere(p_vec, 30_000.0, bevy::color::palettes::css::GREEN);
        
        // Draw the protective radius
        gizmos.circle(
            Isometry3d::new(p_vec, Quat::from_rotation_arc(Vec3::Z, up)),
            zone.radius as f32,
            bevy::color::palettes::css::LIMEGREEN,
        );
    }

    // Active Interceptors
    for abm in interceptor_query.iter() {
        // Draw the interceptor tracking marker
        gizmos.sphere(abm.position_ecef.as_vec3(), 40_000.0, bevy::color::palettes::css::LIME);
    }

    // Impact Prediction (CEP Scatter Visualization)
    if !prediction.coordinates_ecef.is_empty() {
        let mut centroid = DVec3::ZERO;
        
        // Draw individual prediction points in the swarm
        for p in &prediction.coordinates_ecef {
            centroid += *p;
            gizmos.sphere(p.as_vec3(), 15000.0, bevy::color::palettes::css::ORANGE);
        }
        
        centroid /= prediction.coordinates_ecef.len() as f64;
        
        // Calculate 50% radius (CEP)
        let mut distances: Vec<f64> = prediction.coordinates_ecef.iter()
            .map(|p| p.distance(centroid))
            .collect();
        // Sort distances to find median for CEP definition
        // We use sort_by directly since floats can't be purely fully ordered, but this is safe here
        distances.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        
        let cep_radius = distances[distances.len() / 2];
        
        // Draw the CEP boundary circle
        let p_vec = centroid.as_vec3();
        let up = p_vec.normalize();
        let right = up.cross(Vec3::Y).normalize_or_zero();
        let fwd = up.cross(right).normalize_or_zero();
        
        // Draw a multi-segment circle utilizing lines for gizmos approximation
        let segments = 32;
        let first_point = {
            let angle: f32 = 0.0;
            p_vec + (right * angle.cos() + fwd * angle.sin()) * cep_radius as f32
        };
        let mut prev_point: Option<Vec3> = Some(first_point);
        
        for i in 0..=segments {
            let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
            let cur_point = p_vec + (right * angle.cos() + fwd * angle.sin()) * cep_radius as f32;
            
            if let Some(prev) = prev_point {
                gizmos.line(prev, cur_point, bevy::color::palettes::css::RED);
            }
            prev_point = Some(cur_point);
        }
        
        // Close the circle
        if let Some(prev) = prev_point {
            gizmos.line(prev, first_point, bevy::color::palettes::css::RED);
        }
        
        // Pulsate the main centroid marker
        let time_sec = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_secs_f64();
        let pulse = (time_sec * 5.0).sin() as f32; // -1 to 1
        let center_radius = 40000.0 + (pulse * 15000.0);
        
        gizmos.sphere(p_vec, center_radius, bevy::color::palettes::css::RED);
        
        // X marker
        let cross_size = 150000.0;
        gizmos.line(p_vec - right * cross_size, p_vec + right * cross_size, bevy::color::palettes::css::RED);
        gizmos.line(p_vec - fwd * cross_size, p_vec + fwd * cross_size, bevy::color::palettes::css::RED);
    }

    for missile in query.iter() {
        let pos = missile.position_ecef.as_vec3();
        let head_color = match missile.phase {
            FlightPhase::Boost => bevy::color::palettes::css::YELLOW,
            FlightPhase::Ballistic => bevy::color::palettes::css::AQUA,
            FlightPhase::ReEntry => bevy::color::palettes::css::RED,
            FlightPhase::Landed => bevy::color::palettes::css::GREEN,
        };
        // Bevy 0.17 gizmos.sphere(position, radius, color)
        gizmos.sphere(pos, 50000.0, head_color);
        
        // Draw full path with phase-based colors
        if missile.path.len() > 1 {
            for i in 0..missile.path.len()-1 {
                let (p1, phase1) = missile.path[i];
                let (p2, _phase2) = missile.path[i+1];
                
                let color = match phase1 {
                    FlightPhase::Boost => bevy::color::palettes::css::YELLOW,
                    FlightPhase::Ballistic => bevy::color::palettes::css::AQUA,
                    FlightPhase::ReEntry => bevy::color::palettes::css::RED,
                    FlightPhase::Landed => bevy::color::palettes::css::GREEN,
                };
                
                gizmos.line(p1, p2, color);
            }
            // Line to current position
            if let Some((last_pos, last_phase)) = missile.path.last() {
                let color = match last_phase {
                    FlightPhase::Boost => bevy::color::palettes::css::YELLOW,
                    FlightPhase::Ballistic => bevy::color::palettes::css::AQUA,
                    FlightPhase::ReEntry => bevy::color::palettes::css::RED,
                    FlightPhase::Landed => bevy::color::palettes::css::GREEN,
                };
                gizmos.line(*last_pos, pos, color);
            }
        }
    }

    // Reentry bodies (MIRV warheads & decoys)
    for body in reentry_query.iter() {
        let pos = body.position_ecef.as_vec3();
        if body.phase == FlightPhase::Landed { continue; }

        let (head_color, trail_color, size) = match body.body_type {
            ReentryBodyType::Warhead => (
                bevy::color::palettes::css::ORANGE_RED,
                bevy::color::palettes::css::SALMON,
                35000.0,
            ),
            ReentryBodyType::Decoy => (
                bevy::color::palettes::css::GOLD,
                bevy::color::palettes::css::KHAKI,
                25000.0,
            ),
        };

        gizmos.sphere(pos, size, head_color);

        if body.path.len() > 1 {
            for i in 0..body.path.len() - 1 {
                gizmos.line(body.path[i].0, body.path[i + 1].0, trail_color);
            }
            if let Some((last_pos, _)) = body.path.last() {
                gizmos.line(*last_pos, pos, trail_color);
            }
        }
    }
}

fn camera_system(
    mut query: Query<&mut Transform, With<Camera3d>>,
    time: Res<Time>,
    settings: Res<SimulationSettings>,
    mut contexts: EguiContexts,
) {
    // If egui is being interacted with, don't move camera
    if let Ok(ctx) = contexts.ctx_mut() {
        if ctx.is_pointer_over_area() || ctx.wants_pointer_input() || ctx.is_using_pointer() {
            return;
        }
    }

    let mut settings_rotation = settings.theta;
    if !settings.rotation_paused {
        settings_rotation += 0.1 * time.delta_secs();
    }

    // Convert spherical to Cartesian
    let x = settings.zoom_distance * settings.phi.cos() * settings_rotation.sin();
    let y = settings.zoom_distance * settings.phi.sin();
    let z = settings.zoom_distance * settings.phi.cos() * settings_rotation.cos();

    for mut transform in query.iter_mut() {
        transform.translation = Vec3::new(x, y, z);
        transform.look_at(Vec3::ZERO, Vec3::Y);
    }
}

fn solar_lighting_system(
    mut query: Query<&mut Transform, With<Sun>>,
) {
    let now = Utc::now();
    let seconds_since_midnight = now.hour() * 3600 + now.minute() * 60 + now.second();
    let utc_hours = seconds_since_midnight as f64 / 3600.0;
    let doy = now.ordinal() as f64;
    
    // Subsolar point longitude (12:00 UTC = 0 deg)
    let sun_lon_rad = (12.0 - utc_hours) * (15.0f64).to_radians();
    // Subsolar point latitude (declination)
    let dec_rad = (23.44f64).to_radians() * ((360.0 / 365.25) * (doy - 80.0)).to_radians().sin();
    
    let dist = 5.0 * EARTH_RADIUS;
    let x = dist * dec_rad.cos() * sun_lon_rad.cos();
    let y = dist * dec_rad.sin();
    let z = -dist * dec_rad.cos() * sun_lon_rad.sin();
    
    let sun_pos = Vec3::new(x as f32, y as f32, z as f32);
    
    for mut transform in query.iter_mut() {
        transform.translation = sun_pos;
        transform.look_at(Vec3::ZERO, Vec3::Y);
    }
}

// --- Utility Functions ---

pub fn geodetic_to_ecef(lat: f64, lon: f64, alt: f64) -> DVec3 {
    let lat_rad = lat.to_radians();
    let lon_rad = lon.to_radians();
    let r = EARTH_RADIUS + alt;
    DVec3::new(
        r * lat_rad.cos() * lon_rad.cos(),
        r * lat_rad.sin(),
        -r * lat_rad.cos() * lon_rad.sin(),
    )
}

fn destination_point(lat_deg: f64, lon_deg: f64, bearing_deg: f64, distance_m: f64) -> (f64, f64) {
    // Great-circle destination point on a sphere.
    let lat1 = lat_deg.to_radians();
    let lon1 = lon_deg.to_radians();
    let brng = bearing_deg.to_radians();
    let dr = distance_m / EARTH_RADIUS;

    let sin_lat1 = lat1.sin();
    let cos_lat1 = lat1.cos();
    let sin_dr = dr.sin();
    let cos_dr = dr.cos();

    let lat2 = (sin_lat1 * cos_dr + cos_lat1 * sin_dr * brng.cos()).asin();
    let lon2 = lon1
        + (brng.sin() * sin_dr * cos_lat1)
            .atan2(cos_dr - sin_lat1 * lat2.sin());

    let lon2 = ((lon2 + std::f64::consts::PI) % (2.0 * std::f64::consts::PI)) - std::f64::consts::PI;
    (lat2.to_degrees(), lon2.to_degrees())
}


fn input_system(
    keys: Res<ButtonInput<KeyCode>>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut mouse_motion_events: MessageReader<bevy::input::mouse::MouseMotion>,
    mut settings: ResMut<SimulationSettings>,
    mut scroll_evr: MessageReader<bevy::input::mouse::MouseWheel>,
    time: Res<Time>,
    mut contexts: EguiContexts,
) {
    if let Ok(ctx) = contexts.ctx_mut() {
        if ctx.is_pointer_over_area() || ctx.wants_pointer_input() || ctx.is_using_pointer() {
            return;
        }
    }

    if keys.just_pressed(KeyCode::Space) {
        settings.rotation_paused = !settings.rotation_paused;
    }

    // Auto-update theta if not paused
    if !settings.rotation_paused {
        settings.theta += 0.1 * time.delta_secs();
    }

    // Mouse drag rotation
    if buttons.pressed(MouseButton::Left) {
        for event in mouse_motion_events.read() {
            settings.theta -= event.delta.x * 0.005;
            settings.phi += event.delta.y * 0.005;
            // Limit phi to avoid flipping
            settings.phi = settings.phi.clamp(-PI / 2.0 + 0.1, PI / 2.0 - 0.1);
        }
    }

    // Zoom keys
    // Zoom keys
    if keys.pressed(KeyCode::Equal) || keys.pressed(KeyCode::KeyI) {
        settings.zoom_distance -= 100000.0;
    }
    if keys.pressed(KeyCode::Minus) || keys.pressed(KeyCode::KeyO) {
        settings.zoom_distance += 100000.0;
    }

    // Live texture longitude offset tuning
    if keys.just_pressed(KeyCode::BracketRight) {
        settings.texture_lon_offset += 0.5;
        println!("Texture Longitude Offset: {:.2} degrees", settings.texture_lon_offset);
    }
    if keys.just_pressed(KeyCode::BracketLeft) {
        settings.texture_lon_offset -= 0.5;
        println!("Texture Longitude Offset: {:.2} degrees", settings.texture_lon_offset);
    }

    // Mouse scroll zoom
    for ev in scroll_evr.read() {
        settings.zoom_distance -= ev.y * 500000.0;
    }

    // Clamp zoom
    settings.zoom_distance = settings.zoom_distance.clamp(EARTH_RADIUS as f32 * 1.05, EARTH_RADIUS as f32 * 10.0);
}

fn egui_stats_system(
    mut commands: Commands,
    mut contexts: EguiContexts,
    missile_query: Query<(Entity, &Missile)>,
    reentry_query: Query<(Entity, &ReentryBody)>,
    time: Res<Time>,
    window_query: Query<&Window, With<bevy::window::PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    _settings: Res<SimulationSettings>,
    active_specs: Res<ActiveMissileSpecs>,
    error_history: Res<ImpactErrorHistory>,
    flight_history: Res<MissileFlightHistory>,
    registry: Res<MissileRegistry>,
    mut db_initialized: Local<bool>,
    mut mirv_tracker: ResMut<MirvDeploymentTracker>,
) -> Result {
    if time.elapsed_secs() < 0.2 {
        return Ok(());
    }
    let ctx = contexts.ctx_mut()?;
    {
        egui::Window::new("AIBallistic Command Center")
            .default_pos([10.0, 10.0])
            .default_width(360.0)
            .movable(true)
            .collapsible(true)
            .resizable(true)
            .title_bar(true)
            .show(ctx, |ui| {
                use egui_plot::{Line, Plot, PlotPoints};
                ui.style_mut().spacing.button_padding = egui::vec2(10.0, 5.0);

                if let Some((_entity, missile)) = missile_query.iter().next() {
                    let moscow_ecef = geodetic_to_ecef(MOSCOW_LAT, MOSCOW_LON, 0.0);
                    let dist_to_target = missile.position_ecef.distance(moscow_ecef) / 1000.0;
                    let dist_from_start = missile.position_ecef.distance(missile.start_position_ecef) / 1000.0;

                    ui.heading(format!("Model: {}", missile.model.name()));
                    ui.heading(format!("Phase: {:?}", missile.phase));
                    ui.separator();
                    ui.label(format!("Flight Time: {:.1} s", missile.timer));

                    for (label, value) in missile.model.get_stats(missile.position_ecef, missile.velocity_ecef, missile.timer) {
                        ui.label(format!("{}: {}", label, value));
                    }

                    ui.separator();
                    ui.label(format!("Distance from Start: {:.1} km", dist_from_start));
                    ui.label(format!("Distance to Target: {:.1} km", dist_to_target));
                } else {
                    ui.label("No missile active.");
                    if ui.button("Spawn Missile").clicked() {
                        for (ent, _) in reentry_query.iter() {
                            safe_despawn(&mut commands, ent);
                        }
                        mirv_tracker.deployed_missiles.clear();
                        spawn_default_missile(&mut commands, &active_specs);
                    }
                }

                ui.separator();

                // --- Impact Prediction Error ---
                ui.strong("Impact Prediction Error");
                if error_history.data.is_empty() {
                    ui.label("Waiting for radar tracking data...");
                } else {
                    let last = error_history.data.last().unwrap();
                    ui.label(format!("Error: {:.1} km  (t = {:.0} s)", last[1], last[0]));

                    let points: PlotPoints = error_history.data.iter().copied().collect();
                    let line = Line::new("Prediction Error", points)
                        .color(egui::Color32::from_rgb(255, 100, 50));

                    // Least-squares linear regression: y = a + b*x
                    let n = error_history.data.len() as f64;
                    let (sum_x, sum_y, sum_xx, sum_xy) = error_history.data.iter()
                        .fold((0.0, 0.0, 0.0, 0.0), |(sx, sy, sxx, sxy), p| {
                            (sx + p[0], sy + p[1], sxx + p[0] * p[0], sxy + p[0] * p[1])
                        });
                    let denom = n * sum_xx - sum_x * sum_x;
                    let (reg_line, reg_label) = if denom.abs() > 1e-12 && n >= 2.0 {
                        let b = (n * sum_xy - sum_x * sum_y) / denom;
                        let a = (sum_y - b * sum_x) / n;
                        let x0 = error_history.data.first().unwrap()[0];
                        let x1 = error_history.data.last().unwrap()[0];
                        let reg_pts: PlotPoints = vec![[x0, a + b * x0], [x1, a + b * x1]].into_iter().collect();
                        let label = format!("Trend: {:.2} km/s", b);
                        (Some(Line::new("Linear Regression", reg_pts)
                            .color(egui::Color32::from_rgb(150, 200, 255))
                            .style(egui_plot::LineStyle::dashed_dense())), label)
                    } else {
                        (None, String::new())
                    };

                    if !reg_label.is_empty() {
                        ui.label(reg_label);
                    }

                    Plot::new("impact_error_plot")
                        .height(200.0)
                        .x_axis_label("t (s)")
                        .y_axis_label("km")
                        .allow_drag(false)
                        .allow_zoom(false)
                        .allow_scroll(false)
                        .allow_boxed_zoom(false)
                        .show(ui, |plot_ui| {
                            plot_ui.line(line);
                            if let Some(rl) = reg_line {
                                plot_ui.line(rl);
                            }
                        });
                }

                ui.separator();

                // --- Velocity ---
                ui.strong("Velocity");
                if flight_history.velocity.is_empty() {
                    ui.label("Waiting for flight data...");
                } else {
                    let last_v = flight_history.velocity.last().unwrap();
                    let last_m = flight_history.mach.last().map(|m| m[1]).unwrap_or(0.0);
                    ui.label(format!("{:.0} m/s  (Mach {:.1})", last_v[1], last_m));

                    let points: PlotPoints = flight_history.velocity.iter().copied().collect();
                    let line = Line::new("Velocity", points)
                        .color(egui::Color32::from_rgb(100, 200, 255));

                    Plot::new("velocity_plot")
                        .height(200.0)
                        .x_axis_label("t (s)")
                        .y_axis_label("m/s")
                        .allow_drag(false)
                        .allow_zoom(false)
                        .allow_scroll(false)
                        .allow_boxed_zoom(false)
                        .show(ui, |plot_ui| {
                            plot_ui.line(line);
                        });
                }

                ui.separator();

                // --- Altitude ---
                ui.strong("Altitude");
                if flight_history.altitude.is_empty() {
                    ui.label("Waiting for flight data...");
                } else {
                    let last = flight_history.altitude.last().unwrap();
                    ui.label(format!("{:.1} km", last[1]));

                    let points: PlotPoints = flight_history.altitude.iter().copied().collect();
                    let line = Line::new("Altitude", points)
                        .color(egui::Color32::from_rgb(100, 255, 150));

                    Plot::new("altitude_plot")
                        .height(200.0)
                        .x_axis_label("t (s)")
                        .y_axis_label("km")
                        .allow_drag(false)
                        .allow_zoom(false)
                        .allow_scroll(false)
                        .allow_boxed_zoom(false)
                        .show(ui, |plot_ui| {
                            plot_ui.line(line);
                        });
                }

                // --- MIRV / Decoy Status ---
                let warheads: Vec<&ReentryBody> = reentry_query.iter()
                    .map(|(_, b)| b)
                    .filter(|b| b.body_type == ReentryBodyType::Warhead)
                    .collect();
                let decoys: Vec<&ReentryBody> = reentry_query.iter()
                    .map(|(_, b)| b)
                    .filter(|b| b.body_type == ReentryBodyType::Decoy)
                    .collect();
                let total_rv = warheads.len() + decoys.len();

                if total_rv > 0 || active_specs.0.mirv.is_some() {
                    ui.separator();
                    ui.strong("MIRV / Countermeasures");

                    if let Some(cfg) = &active_specs.0.mirv {
                        ui.label(format!("Config: {} RVs + {} decoys", cfg.num_warheads, cfg.num_decoys));
                    }

                    if total_rv > 0 {
                        let active_w = warheads.iter().filter(|b| b.phase != FlightPhase::Landed).count();
                        let active_d = decoys.iter().filter(|b| b.phase != FlightPhase::Landed).count();
                        ui.label(format!(
                            "Active: {} warheads, {} decoys  ({} total objects)",
                            active_w, active_d, active_w + active_d
                        ));

                        let reentry_w = warheads.iter().filter(|b| b.phase == FlightPhase::ReEntry).count();
                        let reentry_d = decoys.iter().filter(|b| b.phase == FlightPhase::ReEntry).count();
                        if reentry_w + reentry_d > 0 {
                            ui.label(format!("  Re-entering: {} RV + {} decoys", reentry_w, reentry_d));
                            if reentry_d > 0 {
                                ui.colored_label(
                                    egui::Color32::from_rgb(255, 200, 50),
                                    "Decoys discriminated (drag divergence)"
                                );
                            }
                        }
                    } else {
                        ui.label("PBV deployment pending...");
                    }
                }
            });

        // --- Missile Database Panel (right side) ---
        let db_window = egui::Window::new("Missile Database")
            .default_width(520.0)
            .movable(true)
            .collapsible(true)
            .resizable(true)
            .title_bar(true);
        let db_window = if !*db_initialized {
            *db_initialized = true;
            db_window.default_pos([ctx.screen_rect().width() - 540.0, 10.0])
        } else {
            db_window
        };
        db_window.show(ctx, |ui| {
                let active_name = active_specs.0.name;
                egui::ScrollArea::vertical().max_height(600.0).show(ui, |ui| {
                    egui::Grid::new("missile_db_grid")
                        .striped(true)
                        .spacing([8.0, 4.0])
                        .show(ui, |ui| {
                            ui.strong("Name");
                            ui.strong("Type");
                            ui.strong("Stages");
                            ui.strong("Total Mass");
                            ui.strong("Payload");
                            ui.strong("Burn (s)");
                            ui.strong("Isp vac");
                            ui.strong("ΔV ideal");
                            ui.strong("MIRV");
                            ui.end_row();

                            for specs in &registry.0 {
                                let is_active = specs.name == active_name;
                                let label = if is_active {
                                    egui::RichText::new(specs.name).strong().color(egui::Color32::from_rgb(100, 255, 150))
                                } else {
                                    egui::RichText::new(specs.name)
                                };
                                ui.label(label);

                                let missile_type = if specs.is_multistage() {
                                    if specs.stages.len() == 2 { "MRBM" } else { "ICBM" }
                                } else if specs.total_mass() > 20000.0 {
                                    "IRBM"
                                } else if specs.total_mass() > 8000.0 {
                                    "MRBM"
                                } else {
                                    "SRBM"
                                };
                                ui.label(missile_type);

                                let n_stages = if specs.is_multistage() {
                                    format!("{}", specs.stages.len())
                                } else {
                                    "1".into()
                                };
                                ui.label(n_stages);

                                ui.label(format!("{:.0} kg", specs.total_mass()));
                                ui.label(format!("{:.0} kg", specs.dry_mass));
                                ui.label(format!("{:.0}", specs.total_burn_time()));

                                let isp_vac = if specs.is_multistage() {
                                    let sum: f64 = specs.stages.iter().map(|s| s.isp_vacuum).sum();
                                    sum / specs.stages.len() as f64
                                } else {
                                    specs.isp_vacuum
                                };
                                ui.label(format!("{:.0} s", isp_vac));

                                let delta_v = compute_ideal_delta_v(specs);
                                ui.label(format!("{:.0} m/s", delta_v));

                                let mirv_label = match &specs.mirv {
                                    Some(cfg) => format!("{}+{}", cfg.num_warheads, cfg.num_decoys),
                                    None => "-".into(),
                                };
                                ui.label(mirv_label);

                                ui.end_row();
                            }
                        });
                });

                ui.separator();
                // Expanded detail for active missile
                ui.strong(format!("Active: {}", active_name));
                let s = &active_specs.0;
                if s.is_multistage() {
                    for (i, stage) in s.stages.iter().enumerate() {
                        ui.horizontal(|ui| {
                            ui.label(format!(
                                "  Stage {}: fuel {:.0} kg, struct {:.0} kg, burn {:.0}s, Isp {:.0}/{:.0}s",
                                i + 1, stage.fuel_mass, stage.dry_mass, stage.burn_time,
                                stage.isp_sea_level, stage.isp_vacuum
                            ));
                        });
                    }
                    ui.label(format!("  RV/Payload: {:.0} kg | Area: {:.2} m²", s.dry_mass, s.area));
                } else {
                    ui.label(format!(
                        "  Fuel: {:.0} kg | Dry: {:.0} kg | Burn: {:.0}s | Isp {:.0}/{:.0}s | Area: {:.2} m²",
                        s.fuel_mass, s.dry_mass, s.burn_time,
                        s.isp_sea_level, s.isp_vacuum, s.area
                    ));
                }

                if let Some(cfg) = &s.mirv {
                    ui.separator();
                    ui.strong("MIRV Configuration");
                    ui.label(format!(
                        "  {} warheads ({:.0} kg, {:.2} m²) + {} decoys ({:.1} kg, {:.2} m²)",
                        cfg.num_warheads, cfg.warhead_mass, cfg.warhead_area,
                        cfg.num_decoys, cfg.decoy_mass, cfg.decoy_area
                    ));
                    ui.label(format!(
                        "  Deploy delay: {:.0}s post-burnout | Spread ΔV: {:.0} m/s",
                        cfg.deploy_delay, cfg.spread_velocity
                    ));
                    let beta_rv = cfg.warhead_mass / (0.3 * cfg.warhead_area);
                    let beta_decoy = cfg.decoy_mass / (0.3 * cfg.decoy_area);
                    ui.label(format!(
                        "  β(RV): {:.0} kg/m² | β(decoy): {:.0} kg/m²  (ratio {:.0}x)",
                        beta_rv, beta_decoy, beta_rv / beta_decoy.max(0.1)
                    ));
                }
            });

        // Mouse Cursor Raycast Tooltip
        let mut cursor_geo_text = "Cursor: Off Earth".to_string();

        if let Some(window) = window_query.iter().next() {
            if let Some(cursor_pos) = window.cursor_position() {
                if let Some((camera, camera_transform)) = camera_query.iter().next() {
                    if let Ok(ray) = camera.viewport_to_world(camera_transform, cursor_pos) {
                        let origin = ray.origin;
                        let dir = *ray.direction;

                        let a: f32 = dir.length_squared();
                        let b: f32 = 2.0 * origin.dot(dir);
                        let c: f32 = origin.length_squared() - (EARTH_RADIUS * EARTH_RADIUS) as f32;

                        let discriminant: f32 = b * b - 4.0 * a * c;
                        if discriminant >= 0.0 {
                            let t = (-b - discriminant.sqrt()) / (2.0 * a);
                            if t > 0.0 {
                                let hit_point = origin + dir * t;

                                let ecef_pos = DVec3::new(hit_point.x as f64, hit_point.y as f64, hit_point.z as f64);
                                let r = ecef_pos.length();
                                let lat = (ecef_pos.y / r).asin().to_degrees();
                                let lon = (-ecef_pos.z).atan2(ecef_pos.x).to_degrees();

                                let lat_str = if lat >= 0.0 { format!("{:.4}° N", lat) } else { format!("{:.4}° S", -lat) };
                                let lon_str = if lon >= 0.0 { format!("{:.4}° E", lon) } else { format!("{:.4}° W", -lon) };

                                cursor_geo_text = format!("{}, {}", lat_str, lon_str);
                            }
                        }
                    }
                }

                if cursor_geo_text != "Cursor: Off Earth" {
                    egui::Area::new(egui::Id::new("cursor_tooltip"))
                        .fixed_pos(egui::pos2(cursor_pos.x + 15.0, cursor_pos.y + 15.0))
                        .order(egui::Order::Tooltip)
                        .show(ctx, |ui| {
                            egui::Frame::popup(ui.style()).show(ui, |ui| {
                                ui.label(cursor_geo_text);
                            });
                        });
                }
            }
        }
    }
    Ok(())
}

fn mirv_deployment_system(
    mut commands: Commands,
    missile_query: Query<(Entity, &Missile)>,
    mut tracker: ResMut<MirvDeploymentTracker>,
    active_specs: Res<ActiveMissileSpecs>,
) {
    let mirv_config = match &active_specs.0.mirv {
        Some(cfg) => *cfg,
        None => return,
    };

    for (entity, missile) in missile_query.iter() {
        if tracker.deployed_missiles.contains(&entity) { continue; }
        
        let total_bt = active_specs.0.total_burn_time();
        if missile.timer < total_bt + mirv_config.deploy_delay { continue; }
        if missile.phase == FlightPhase::Landed { continue; }

        tracker.deployed_missiles.push(entity);

        let pos = missile.position_ecef;
        let vel = missile.velocity_ecef;
        let up = pos.normalize();
        let vel_dir = vel.normalize();
        let side = vel_dir.cross(up).normalize();

        let total_objects = mirv_config.num_warheads + mirv_config.num_decoys;
        
        for i in 0..total_objects {
            let is_warhead = i < mirv_config.num_warheads;
            let angle = (i as f64 / total_objects as f64) * std::f64::consts::TAU;
            let spread_dir = (side * angle.cos() + up * angle.sin() * 0.3).normalize();
            let dv = spread_dir * mirv_config.spread_velocity;

            let (mass, area, body_type) = if is_warhead {
                (mirv_config.warhead_mass, mirv_config.warhead_area, ReentryBodyType::Warhead)
            } else {
                (mirv_config.decoy_mass, mirv_config.decoy_area, ReentryBodyType::Decoy)
            };

            commands.spawn(ReentryBody {
                body_type,
                position_ecef: pos + spread_dir * 100.0,
                velocity_ecef: vel + dv,
                mass,
                area,
                path: Vec::new(),
                phase: FlightPhase::Ballistic,
                parent_missile: Some(entity),
            });
        }
    }
}

fn reentry_body_physics_system(
    time: Res<Time>,
    settings: Res<SimulationSettings>,
    mut query: Query<&mut ReentryBody>,
) {
    let dt = time.delta_secs() * settings.time_scale;

    for mut body in query.iter_mut() {
        if body.phase == FlightPhase::Landed { continue; }

        let r = body.position_ecef.length();
        if r < EARTH_RADIUS && body.path.len() > 2 {
            body.velocity_ecef = DVec3::ZERO;
            body.phase = FlightPhase::Landed;
            continue;
        }

        let gravity_dir = -body.position_ecef.normalize();
        let gravity_accel = gravity_dir * (GRAVITY_CONSTANT / (r * r));

        let coriolis_accel = if settings.coriolis_enabled {
            -2.0 * EARTH_OMEGA.cross(body.velocity_ecef)
        } else { DVec3::ZERO };
        let centrifugal_accel = if settings.centrifugal_enabled {
            -EARTH_OMEGA.cross(EARTH_OMEGA.cross(body.position_ecef))
        } else { DVec3::ZERO };

        let altitude = (r - EARTH_RADIUS).max(0.0);
        let (rho, sound_speed, _) = get_isa_properties(altitude);
        let speed = body.velocity_ecef.length();
        let mach = speed / sound_speed;
        let cd = get_mach_drag(mach);
        let drag_force = -0.5 * rho * speed * speed * cd * body.area;
        let drag_accel = if speed > 1e-3 {
            (body.velocity_ecef.normalize() * drag_force) / body.mass
        } else { DVec3::ZERO };

        let phase = if altitude > 120000.0 || body.velocity_ecef.dot(body.position_ecef) >= 0.0 {
            FlightPhase::Ballistic
        } else {
            FlightPhase::ReEntry
        };
        body.phase = phase;

        let total_accel = gravity_accel + coriolis_accel + centrifugal_accel + drag_accel;
        body.velocity_ecef += total_accel * dt as f64;
        let new_vel = body.velocity_ecef;
        body.position_ecef += new_vel * dt as f64;

        let pos_vec3 = body.position_ecef.as_vec3();
        let btype = body.body_type;
        let should_push = body.path.is_empty() || pos_vec3.distance(body.path.last().unwrap().0) > 10000.0;
        if should_push {
            body.path.push((pos_vec3, btype));
        }
    }
}

fn spawn_default_missile(commands: &mut Commands, active_specs: &ActiveMissileSpecs) {
    let tehran_ecef = geodetic_to_ecef(35.6892, 51.3890, 10.0);
    let specs = active_specs.0.clone();
    let initial_mass = specs.total_mass();
    commands.spawn(Missile {
        position_ecef: tehran_ecef,
        start_position_ecef: tehran_ecef,
        velocity_ecef: DVec3::ZERO,
        mass: initial_mass,
        timer: 0.0,
        phase: FlightPhase::Boost,
        path: Vec::new(),
        model: Box::new(BallisticMissilePhysics::new(specs)),
    });
}

fn earth_alignment_system(
    settings: Res<SimulationSettings>,
    mut query: Query<&mut Transform, With<Earth>>,
) {
    if settings.is_changed() {
        for mut transform in query.iter_mut() {
            transform.rotation = Quat::from_rotation_y(settings.texture_lon_offset.to_radians()) *
            Quat::from_mat3(&bevy::math::Mat3::from_cols(
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(0.0, 1.0, 0.0),
            ));
        }
    }
}
