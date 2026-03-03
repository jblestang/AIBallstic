fn main() {
    let lat = 44.07_f64;
    let lon = 24.31_f64;
    let lat_rad = lat.to_radians();
    let lon_rad = lon.to_radians();
    let r = 1.0;
    
    // Our ECEF: Y is UP (North Pole)
    let x = r * lat_rad.cos() * lon_rad.cos();
    let y = r * lat_rad.sin();
    let z = r * lat_rad.cos() * lon_rad.sin();
    
    println!("ECEF: x={}, y={}, z={}", x, y, z);
    
    // If UV mapping starts at -Z and wraps counter-clockwise:
    let uv_u = 0.5 + z.atan2(x) / (2.0 * std::f64::consts::PI);
    println!("UV U coordinate: {}", uv_u);
}
