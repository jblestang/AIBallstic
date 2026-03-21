//! CLI: find **in-sim** fuel mass (game physics) to land within a great-circle radius of the
//! default target (Diego Garcia). For balancing / what-if only — not a real-world performance claim.
//!
//! Run from the repo root: `cargo run --release --bin fuel_sweep`

use aiballistic::missiles::{geodetic_to_ecef, get_missile_registry, MissileSpecs};
use aiballistic::sim::{simulate_until_ground_impact, SimParams};

const DEFAULT_MISSILE: &str = "Khorramshahr-4 (Kheibar)";
const TEHRAN_LAT: f64 = 35.6892;
const TEHRAN_LON: f64 = 51.3890;
const TEHRAN_ALT_M: f64 = 10.0;

fn usage() -> ! {
    eprintln!(
        "\
fuel_sweep — search fuel_mass for in-sim hits near default target (Diego Garcia)

Usage:
  cargo run --release --bin fuel_sweep -- [options]

Options:
  --name <str>           Missile entry name in assets/missiles.json (default: {DEFAULT_MISSILE})
  --radius-km <f64>      Great-circle CEP threshold in km (default: 100)
  --step-kg <f64>        Fuel search step in kg (default: 5000)
  --min-fuel-kg <f64>    Lower bound when decreasing from baseline (default: 500)
  --max-fuel-kg <f64>    Upper search bound (default: 2e6)
  --dt <f64>             Fixed integrator step in sim seconds (default: 0.02)
  --max-time-s <f32>     Abort integration after this sim time (default: 20000)
  --no-coriolis          Disable Coriolis (matches toggles in the UI)
  --no-centrifugal       Disable centrifugal term

Notes:
  - Uses the same thrust model as the game: burn_rate = fuel_mass / burn_time (fixed burn_time).
  - \"Hit\" is not guaranteed monotonic in fuel; this tool steps fuel and reports the first
    threshold crossing when increasing from baseline, or the lowest fuel (by step) when decreasing.
"
    );
    std::process::exit(2);
}

fn parse_f64(a: &str) -> f64 {
    a.parse().unwrap_or_else(|_| {
        eprintln!("Invalid number: {a}");
        std::process::exit(2);
    })
}

fn parse_f32(a: &str) -> f32 {
    a.parse().unwrap_or_else(|_| {
        eprintln!("Invalid number: {a}");
        std::process::exit(2);
    })
}

fn find_specs<'a>(registry: &'a [MissileSpecs], name: &str) -> &'a MissileSpecs {
    registry
        .iter()
        .find(|s| s.name == name)
        .unwrap_or_else(|| {
            eprintln!("Unknown missile name: {name}");
            eprintln!("Available:");
            for s in registry {
                eprintln!("  {}", s.name);
            }
            std::process::exit(1);
        })
}

fn main() {
    let mut missile_name = DEFAULT_MISSILE.to_string();
    let mut radius_km = 100.0_f64;
    let mut step_kg = 5_000.0_f64;
    let mut min_fuel_kg = 500.0_f64;
    let mut max_fuel_kg = 2_000_000.0_f64;
    let mut dt = 0.02_f64;
    let mut max_time_s = 20_000.0_f32;
    let mut coriolis = true;
    let mut centrifugal = true;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => usage(),
            "--name" => missile_name = args.next().unwrap_or_else(|| usage()),
            "--radius-km" => radius_km = parse_f64(&args.next().unwrap_or_else(|| usage())),
            "--step-kg" => step_kg = parse_f64(&args.next().unwrap_or_else(|| usage())),
            "--min-fuel-kg" => min_fuel_kg = parse_f64(&args.next().unwrap_or_else(|| usage())),
            "--max-fuel-kg" => max_fuel_kg = parse_f64(&args.next().unwrap_or_else(|| usage())),
            "--dt" => dt = parse_f64(&args.next().unwrap_or_else(|| usage())),
            "--max-time-s" => max_time_s = parse_f32(&args.next().unwrap_or_else(|| usage())),
            "--no-coriolis" => coriolis = false,
            "--no-centrifugal" => centrifugal = false,
            other => {
                eprintln!("Unknown argument: {other}");
                usage();
            }
        }
    }

    if step_kg <= 0.0 || max_fuel_kg <= min_fuel_kg {
        eprintln!("Invalid step or fuel bounds.");
        std::process::exit(2);
    }

    let registry = get_missile_registry();
    let base = find_specs(&registry, &missile_name).clone();

    if base.is_multistage() {
        eprintln!("fuel_sweep currently supports single-stage JSON entries only.");
        std::process::exit(1);
    }

    let launch = geodetic_to_ecef(TEHRAN_LAT, TEHRAN_LON, TEHRAN_ALT_M);
    let params = SimParams {
        dt,
        coriolis_enabled: coriolis,
        centrifugal_enabled: centrifugal,
        max_time_s,
    };

    let radius_m = radius_km * 1000.0;
    let base_fuel = base.fuel_mass;

    let run = |fuel: f64| {
        let mut s = base.clone();
        s.fuel_mass = fuel;
        simulate_until_ground_impact(s, launch, &params)
    };

    let baseline = run(base_fuel);
    println!("=== In-sim fuel sweep (fictional / balancing tool) ===");
    println!("Missile: {}", base.name);
    println!("Launch: Tehran ({TEHRAN_LAT}, {TEHRAN_LON}) alt {TEHRAN_ALT_M} m");
    println!("Target miss threshold: {radius_km} km great-circle");
    println!("Baseline fuel_mass: {base_fuel:.1} kg");
    println!(
        "Baseline: landed={}  t={:.1}s  miss={:.3} km  impact=({:.4}°, {:.4}°)",
        baseline.landed,
        baseline.flight_time_s,
        baseline.miss_distance_m / 1000.0,
        baseline.impact_lat_deg,
        baseline.impact_lon_deg
    );

    let hits = |fuel: f64| {
        let r = run(fuel);
        r.landed && r.miss_distance_m <= radius_m
    };

    let mut solution: Option<f64> = None;

    if hits(base_fuel) {
        let mut f = base_fuel;
        while f - step_kg >= min_fuel_kg && hits(f - step_kg) {
            f -= step_kg;
        }
        solution = Some(f);
    } else {
        let mut f = base_fuel;
        let mut best_miss_km = if baseline.landed {
            baseline.miss_distance_m / 1000.0
        } else {
            f64::INFINITY
        };
        let mut best_fuel = base_fuel;
        while f + step_kg <= max_fuel_kg {
            f += step_kg;
            let r = run(f);
            if r.landed {
                let mk = r.miss_distance_m / 1000.0;
                if mk < best_miss_km {
                    best_miss_km = mk;
                    best_fuel = f;
                }
                if mk * 1000.0 <= radius_m {
                    solution = Some(f);
                    break;
                }
            }
        }
        if solution.is_none() && best_miss_km.is_finite() {
            println!();
            println!(
                "Best landed case in search (coarse): fuel_mass={best_fuel:.1} kg  miss={best_miss_km:.1} km"
            );
        }
    }

    match solution {
        Some(fuel) => {
            let r = run(fuel);
            println!();
            println!("First coarse solution (step {step_kg} kg):");
            println!("  fuel_mass: {fuel:.1} kg  (delta {:+.1} kg vs baseline)", fuel - base_fuel);
            println!(
                "  landed={}  t={:.1}s  miss={:.3} km  impact=({:.4}°, {:.4}°)",
                r.landed,
                r.flight_time_s,
                r.miss_distance_m / 1000.0,
                r.impact_lat_deg,
                r.impact_lon_deg
            );
        }
        None => {
            println!();
            println!(
                "No fuel in [{min_fuel_kg}, {max_fuel_kg}] (step {step_kg}) landed within {radius_km} km."
            );
            println!("Try larger --max-fuel-kg, looser --radius-km, or smaller --dt.");
            std::process::exit(1);
        }
    }
}
