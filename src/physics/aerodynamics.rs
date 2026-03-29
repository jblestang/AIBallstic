//! Empirical drag coefficient vs Mach for a generic axisymmetric body.

/// Empirical **C_d(M)** (slender body, game-tuned): transonic peak, supersonic decay, hypersonic plateau.
pub fn get_mach_drag(mach: f64) -> f64 {
    const M: &[f64] = &[
        0.0, 0.35, 0.70, 0.88, 1.0, 1.12, 1.35, 1.7, 2.2, 3.0, 4.5, 7.0, 12.0, 25.0,
    ];
    const CD: &[f64] = &[
        0.16, 0.175, 0.20, 0.30, 0.52, 0.62, 0.48, 0.38, 0.31, 0.26, 0.23, 0.21, 0.195, 0.185,
    ];
    debug_assert_eq!(M.len(), CD.len());

    let m = mach.max(0.0);
    if m >= *M.last().unwrap() {
        return *CD.last().unwrap();
    }

    for i in 0..M.len() - 1 {
        if m <= M[i + 1] {
            let t = (m - M[i]) / (M[i + 1] - M[i]);
            return CD[i] + t * (CD[i + 1] - CD[i]);
        }
    }
    *CD.last().unwrap()
}
