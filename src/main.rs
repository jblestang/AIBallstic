mod missiles;

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use glam::{DVec3, Vec3};
use nalgebra::{SMatrix, SVector};
use std::f32::consts::PI;
use missiles::*;
use chrono::{Datelike, Timelike, Utc};

// Constants
// Real-world alignment is handled in missiles.rs constants

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

    let mut selected_specs = registry.first().copied().expect("Missile registry is empty!");
    if let Some(pos) = args.iter().position(|arg| arg == "--missile" || arg == "-m") {
        if let Some(target_name) = args.get(pos + 1) {
            if let Some(specs) = registry.iter().find(|s| s.name.to_lowercase().contains(&target_name.to_lowercase())) {
                selected_specs = *specs;
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

    App::new()
        .add_plugins(DefaultPlugins.set(ImagePlugin::default_nearest()))
        .add_plugins(EguiPlugin::default())
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
            texture_lon_offset: -90.0, // Compensate for Bevy's U sphere anchor wrapping 90deg left
        })
        .init_resource::<TrackedMissileState>()
        .init_resource::<ImpactPrediction>()
        .init_resource::<ABMLaunchQueue>()
        .add_systems(Startup, setup)
        .add_systems(Update, (
            egui_stats_system,
            (
                physics_system, trajectory_system, camera_system, input_system, 
                solar_lighting_system, radar_scan_system, impact_prediction_system, earth_alignment_system
            ).after(egui_stats_system),
            (
                abm_c2_system, spawn_abm_system, abm_guidance_system, abm_kill_system, explosion_system
            ).after(egui_stats_system),
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
            unlit: false,
            ..default()
        })),
        Earth,
        Transform::default(),
    ));

    // Light (Sun)
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0 * EARTH_RADIUS as f32, 0.0, 2.0 * EARTH_RADIUS as f32).looking_at(Vec3::ZERO, Vec3::Y),
        Sun
    ));

    // Radar Station (E.g., Romania / Deveselu for Aegis Ashore, or generic European location)
    // Let's place it somewhere in Europe to track the Tehran->Moscow flight.
    // Coordinates: 44.07, 24.31 (Deveselu, Romania)
    commands.spawn(RadarStation {
        position_ecef: geodetic_to_ecef(44.07, 24.31, 100.0),
        range: 5_000_000.0, // 5000 km
        scan_timer: 0.0,
    });

    // Defended Zone (Moscow)
    commands.spawn(DefendedZone {
        position_ecef: geodetic_to_ecef(55.7558, 37.6173, 0.0),
        radius: 300_000.0, // 300 km point-defense radius
    });

    // Launch from TEHRAN, IR
    // Geodetic: 35.6892° N, 51.3890° E
    let tehran_ecef = geodetic_to_ecef(35.6892, 51.3890, 10.0);
    let specs = active_specs.0;
    
    commands.spawn(Missile {
        position_ecef: tehran_ecef,
        start_position_ecef: tehran_ecef,
        velocity_ecef: DVec3::ZERO,
        mass: specs.dry_mass + specs.fuel_mass,
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
        // Sample points for the 3D drawing system.
        if missile.timer % 1.0 < dt { 
            let pos = missile.position_ecef;
            let phase = missile.phase;
            missile.path.push((pos.as_vec3(), phase));
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

fn impact_prediction_system(
    tracked_state: Res<TrackedMissileState>,
    mut prediction: ResMut<ImpactPrediction>,
    settings: Res<SimulationSettings>,
) {
    // Only run the prediction if the EKF state has been updated.
    if !tracked_state.is_changed() || tracked_state.ekf_state.is_none() {
        return;
    }
    
    let ekf = tracked_state.ekf_state.unwrap();
    let base_pos = DVec3::new(ekf[0], ekf[1], ekf[2]);
    let base_vel = DVec3::new(ekf[3], ekf[4], ekf[5]);
    
    // Clear previous predictions
    prediction.coordinates_ecef.clear();
    
    let n_simulations = 50;
    use rand::Rng;
    let mut rng = rand::thread_rng();

    for _ in 0..n_simulations {
        // Fast-forward fixed time-step simulation
        let dt: f32 = 1.0; 
        
        // Inject per-trajectory variance to build a scatter plot.
        // E.g., slightly different atmospheric responses or RV estimations.
        let mut pos = base_pos + DVec3::new(
            rng.gen_range(-500.0..500.0),
            rng.gen_range(-500.0..500.0),
            rng.gen_range(-500.0..500.0)
        );
        let mut vel = base_vel + DVec3::new(
            rng.gen_range(-10.0..10.0),
            rng.gen_range(-10.0..10.0),
            rng.gen_range(-10.0..10.0)
        );
        
        // Randomize the Ballistic Coefficient bounds
        let est_mass = rng.gen_range(400.0..600.0); // RV mass between 400kg - 600kg
        let est_area = rng.gen_range(0.4..0.6);   // RV cross-section
        
        let mut predicted_alt = (pos.length() - EARTH_RADIUS).max(0.0);
        let mut iterations = 0; 
        
        while predicted_alt > 0.0 && iterations < 5000 {
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
        
        prediction.coordinates_ecef.push(pos);
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
    interceptor_query: Query<&ABMInterceptor>,
) {
    if prediction.coordinates_ecef.is_empty() { return; }
    
    // Determine the centroid of the swarm
    let mut centroid = DVec3::ZERO;
    for &pt in &prediction.coordinates_ecef {
        centroid += pt;
    }
    centroid /= prediction.coordinates_ecef.len() as f64;
    
    for zone in defended_zones.iter() {
        if centroid.distance(zone.position_ecef) <= zone.radius {
            // Threat calculated to land within defended zone!
            
            for (missile_entity, missile) in missile_query.iter() {
                if missile.phase == FlightPhase::Landed { continue; }
                
                // Do not launch if there is already an active interceptor
                let mut already_targeted = false;
                for int in interceptor_query.iter() {
                    if int.target_entity == missile_entity { already_targeted = true; break; }
                }
                if already_targeted { continue; }
                
                // Only launch if the target is within 500km of the defended zone
                let dist_to_zone = missile.position_ecef.distance(zone.position_ecef);
                if dist_to_zone < 500_000.0 {
                    let battery_pos = zone.position_ecef + (zone.position_ecef.normalize() * 100.0); // Surface launch
                    launch_queue.0.push(SpawnABMEvent {
                        target_entity: missile_entity,
                        battery_pos,
                    });
                }
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
    tracked_state: Res<TrackedMissileState>,
) {
    let dt = time.delta_secs() * settings.time_scale;
    
    for (int_ent, mut abm, mut transform) in interceptor_query.iter_mut() {
        let target_opt = missile_query.iter().find(|(e, _)| *e == abm.target_entity);
        if let Some((_, target_missile)) = target_opt {
            
            if target_missile.phase == FlightPhase::Landed {
                commands.entity(int_ent).despawn();
                continue;
            }
            
            // Proportional Navigation
            let target_pos = tracked_state.position_ecef.unwrap_or(target_missile.position_ecef);
            let target_vel = tracked_state.velocity_ecef.unwrap_or(target_missile.velocity_ecef);
            
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
            
            // Gravity
            let r = abm.position_ecef.length();
            let gravity = -abm.position_ecef.normalize() * (GRAVITY_CONSTANT / (r * r));
            accel += gravity;
            
            // Thrust
            let speed = abm.velocity_ecef.length();
            if speed < 5000.0 {
                let thrust_dir = if speed > 1.0 { abm.velocity_ecef.normalize() } else { los_dir };
                accel += thrust_dir * (30.0 * 9.81); 
            }
            
            abm.velocity_ecef += accel * dt as f64;
            let delta = abm.velocity_ecef * dt as f64;
            abm.position_ecef += delta;
            transform.translation = abm.position_ecef.as_vec3();
            
        } else {
            commands.entity(int_ent).despawn();
        }
    }
}

fn abm_kill_system(
    mut commands: Commands,
    interceptor_query: Query<(Entity, &ABMInterceptor)>,
    missile_query: Query<(Entity, &Missile)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (int_ent, abm) in interceptor_query.iter() {
        if let Ok((missile_ent, missile)) = missile_query.get(abm.target_entity) {
            let dist = abm.position_ecef.distance(missile.position_ecef);
            if dist < abm.kill_radius {
                let alt = (missile.position_ecef.length() - EARTH_RADIUS) / 1000.0;
                println!("💥 INTERCEPT SUCCESSFUL! Threat neutralized at {:.1} km altitude.", alt);
                
                commands.spawn((
                    Mesh3d(meshes.add(Sphere::new(100_000.0).mesh().uv(32, 16))), 
                    MeshMaterial3d(materials.add(StandardMaterial {
                        base_color: bevy::color::Color::srgb(1.0, 0.5, 0.0), // Orange fireball
                        unlit: true,
                        ..default()
                    })),
                    Transform::from_translation(abm.position_ecef.as_vec3()),
                    Explosion { timer: 0.0, max_time: 2.0 }
                ));
                
                commands.entity(missile_ent).despawn();
                commands.entity(int_ent).despawn();
            }
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
            commands.entity(e).despawn();
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
    radar_query: Query<&RadarStation>,
    defended_zones: Query<&DefendedZone>,
    interceptor_query: Query<&ABMInterceptor>,
    prediction: Res<ImpactPrediction>,
    settings: Res<SimulationSettings>,
) {
    // Draw Meridians (Longitude lines every 15 degrees)
    for lon in (0..360).step_by(15) {
        let lon_rad = (lon as f32).to_radians();
        let normal = Vec3::new(lon_rad.sin(), 0.0, -lon_rad.cos());
        gizmos.circle(
            Isometry3d::new(Vec3::ZERO, Quat::from_rotation_arc(Vec3::Z, normal)),
            EARTH_RADIUS as f32 * 1.002, // Slightly above surface
            bevy::color::Color::srgba(1.0, 1.0, 1.0, 0.4),
        );
    }
    
    // Draw Parallels (Latitude lines every 15 degrees)
    for lat in (-75..=75).step_by(15) {
        let lat_rad = (lat as f32).to_radians();
        let center_y = EARTH_RADIUS as f32 * lat_rad.sin();
        let radius = EARTH_RADIUS as f32 * lat_rad.cos();
        gizmos.circle(
            Isometry3d::new(Vec3::new(0.0, center_y, 0.0), Quat::from_rotation_arc(Vec3::Z, Vec3::Y)),
            radius * 1.002,
            bevy::color::Color::srgba(1.0, 1.0, 1.0, 0.4),
        );
    }

    // Target Markers
    // Tehran - RED
    let tehran = geodetic_to_ecef(35.6892, 51.3890, 0.0);
    gizmos.sphere(tehran.as_vec3(), 50000.0, bevy::color::palettes::css::RED);

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
    let x = dist * dec_rad.cos() * sun_lon_rad.sin();
    let y = dist * dec_rad.sin();
    let z = dist * dec_rad.cos() * sun_lon_rad.cos();
    
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
        r * lat_rad.cos() * lon_rad.sin(),
        r * lat_rad.sin(),
        r * lat_rad.cos() * lon_rad.cos(),
    )
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
    time: Res<Time>,
    window_query: Query<&Window, With<bevy::window::PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    _settings: Res<SimulationSettings>,
    active_specs: Res<ActiveMissileSpecs>,
) {
    // Avoid panics on first frames if egui isn't
    // fully initialized with fonts
    if time.elapsed_secs() < 0.2 {
        return;
    }
    if let Ok(ctx) = contexts.ctx_mut() {
        egui::Window::new("AIBallistic Command Center 🚀")
            .fixed_pos([10.0, 10.0])
            .default_width(320.0)
            .interactable(true)
            .show(ctx, |ui| {
                ui.style_mut().spacing.button_padding = egui::vec2(10.0, 5.0);
                
                if let Some((_entity, missile)) = missile_query.iter().next() {
                    let moscow_ecef = geodetic_to_ecef(MOSCOW_LAT, MOSCOW_LON, 0.0);
                    let dist_to_target = missile.position_ecef.distance(moscow_ecef) / 1000.0;
                    let dist_from_start = missile.position_ecef.distance(missile.start_position_ecef) / 1000.0;
                    
                    ui.vertical(|ui| {
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
                    });

                } else {
                    ui.label("No missile active.");
                    if ui.button("Spawn Missile").clicked() {
                        spawn_default_missile(&mut commands, &active_specs);
                    }
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
                                
                                // Standard ECEF to Lat/Lon
                                let ecef_pos = DVec3::new(hit_point.x as f64, hit_point.y as f64, hit_point.z as f64);
                                let r = ecef_pos.length();
                                let lat = (ecef_pos.y / r).asin().to_degrees();
                                // We use x.atan2(z) because x=sin(lon), z=cos(lon).
                                let lon = ecef_pos.x.atan2(ecef_pos.z).to_degrees();
                                
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
}

fn spawn_default_missile(commands: &mut Commands, active_specs: &ActiveMissileSpecs) {
    let tehran_ecef = geodetic_to_ecef(35.6892, 51.3890, 10.0);
    let specs = active_specs.0;
    commands.spawn(Missile {
        position_ecef: tehran_ecef,
        start_position_ecef: tehran_ecef,
        velocity_ecef: DVec3::ZERO,
        mass: specs.dry_mass + specs.fuel_mass,
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
            transform.rotation = Quat::from_rotation_y(settings.texture_lon_offset.to_radians());
        }
    }
}
