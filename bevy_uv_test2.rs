fn main() {
    // Let's test the Greenwich Meridian (Lon 0, Lat 0)
    let lat = 0.0_f64;
    let lon = 0.0_f64;
    let r = 1.0;
    
    // Standard ECEF
    let x = r * lat.to_radians().cos() * lon.to_radians().cos();
    let y = r * lat.to_radians().sin();
    let z = r * lat.to_radians().cos() * lon.to_radians().sin();
    
    println!("Greenwich ECEF: x={}, y={}, z={}", x, y, z);
    
    // In Bevy's Sphere primitive, the UV mapping formula is:
    // u = 0.5 + atan2(z, x) / 2PI
    let u_greenwich = 0.5 + z.atan2(x) / (2.0 * std::f64::consts::PI);
    println!("Greenwich UV: u={}", u_greenwich);
    
    // A standard equirectangular texture has Greenwich centered at U=0.5
    // so Bevy's mapping actually puts X=1.0 right at the center of the texture!
    // This perfectly matches standard Earth textures where Europe is in the middle.
    
    // Now let's test Tehran (35.6892, 51.3890)
    let t_lat = 35.6892_f64;
    let t_lon = 51.3890_f64;
    let tx = r * t_lat.to_radians().cos() * t_lon.to_radians().cos();
    let tz = r * t_lat.to_radians().cos() * t_lon.to_radians().sin();
    let u_tehran = 0.5 + tz.atan2(tx) / (2.0 * std::f64::consts::PI);
    
    // 51 degrees East should be roughly 0.5 + (51/360) = 0.641
    println!("Tehran UV: u={} (expected 0.641)", u_tehran);
}
