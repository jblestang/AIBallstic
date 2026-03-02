mod missiles;

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use glam::{DVec3, Vec3};
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
    pub path: Vec<Vec3>,
    pub model: Box<dyn PhysicsModel>,
}

#[derive(Component)]
pub struct Sun;

#[derive(Component)]
struct Earth;

#[derive(Resource)]
pub struct ActiveMissileSpecs(pub MissileSpecs);

#[derive(Resource)]
struct SimulationSettings {
    time_scale: f32,
    rotation_paused: bool,
    zoom_distance: f32,
    theta: f32, // Horizontal angle
    phi: f32,   // Vertical angle
    coriolis_enabled: bool,
    centrifugal_enabled: bool,
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

    let mut selected_specs = KHORRAMSHAHR_4_SPECS;
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
        })
        .add_systems(Startup, setup)
        .add_systems(Update, (
            egui_stats_system,
            (physics_system, trajectory_system, camera_system, input_system, solar_lighting_system)
                .after(egui_stats_system),
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
        // Texture Alignment:
        // Texture North: +Z, Lon 0: -X, Lat 90E: -Y
        // Physics North: +Y, Lon 0: +Z, Lat 90E: +X
        // Matrix to map: R*Z=Y, R*(-X)=Z, R*(-Y)=X
        Transform::from_rotation(Quat::from_mat3(&bevy::math::Mat3::from_cols(
            Vec3::new(0.0, 0.0, -1.0), // R*X = -Z
            Vec3::new(-1.0, 0.0, 0.0), // R*Y = -X
            Vec3::new(0.0, 1.0, 0.0),  // R*Z = +Y
        ))),
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

        // Global acceleration terms (Gravitational + Fictitious)
        let gravity_dir = -missile.position_ecef.normalize();
        let gravity_accel = gravity_dir * (GRAVITY_CONSTANT / (r * r));

        let v_ecef = missile.velocity_ecef;
        let r_ecef = missile.position_ecef;
        
        let coriolis_accel = if settings.coriolis_enabled {
            -2.0 * EARTH_OMEGA.cross(v_ecef)
        } else {
            DVec3::ZERO
        };
        
        let centrifugal_accel = if settings.centrifugal_enabled {
            -EARTH_OMEGA.cross(EARTH_OMEGA.cross(r_ecef))
        } else {
            DVec3::ZERO
        };

        // Pluggable Model Specific Acceleration (Thrust, Drag, Guidance, etc.)
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

        // Integration
        let total_accel = gravity_accel + coriolis_accel + centrifugal_accel + model_accel;
        missile.velocity_ecef += total_accel * dt as f64;
        let vel = missile.velocity_ecef;
        missile.position_ecef += vel * dt as f64;
        
        // Track path
        if missile.timer % 1.0 < dt { 
            let pos = missile.position_ecef;
            missile.path.push(pos.as_vec3());
        }
    }
}

fn trajectory_system(
    mut gizmos: Gizmos,
    query: Query<&Missile>,
) {
    // Target Markers
    // Tehran - RED
    let tehran = geodetic_to_ecef(35.6892, 51.3890, 0.0).as_vec3();
    gizmos.sphere(tehran, 50000.0, bevy::color::palettes::css::RED);

    // Moscow - MAGENTA
    let moscow = geodetic_to_ecef(MOSCOW_LAT, MOSCOW_LON, 0.0).as_vec3();
    gizmos.sphere(moscow, 50000.0, bevy::color::palettes::css::MAGENTA);

    for missile in query.iter() {
        let pos = missile.position_ecef.as_vec3();
        // Bevy 0.17 gizmos.sphere(position, radius, color)
        gizmos.sphere(pos, 50000.0, bevy::color::palettes::basic::RED);
        
        // Draw full path
        if missile.path.len() > 1 {
            for i in 0..missile.path.len()-1 {
                gizmos.line(missile.path[i], missile.path[i+1], bevy::color::palettes::css::ORANGE);
            }
            // Line to current position
            if let Some(last) = missile.path.last() {
                gizmos.line(*last, pos, bevy::color::palettes::css::ORANGE);
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
    let z = dist * dec_rad.cos() * sun_lon_rad.sin();
    
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
        r * lat_rad.cos() * lon_rad.sin(),
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
    if keys.pressed(KeyCode::Equal) || keys.pressed(KeyCode::KeyI) {
        settings.zoom_distance -= 100000.0;
    }
    if keys.pressed(KeyCode::Minus) || keys.pressed(KeyCode::KeyO) {
        settings.zoom_distance += 100000.0;
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
    _window_query: Query<&Window, With<bevy::window::PrimaryWindow>>,
    _settings: Res<SimulationSettings>,
    active_specs: Res<ActiveMissileSpecs>,
) {
    // Avoid panics on first frames if egui isn't fully initialized with fonts
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
