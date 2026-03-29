# Physical model (AIBallistic)

This document summarizes the physics implemented under `src/physics/`. The simulation targets **pedagogical / game fidelity**: Earth–fixed ECEF integration with a realistic gravity field and a standard atmosphere, not a full missile six‑DOF or operational trajectory tool.

## Reference frame

- **Position and velocity** are expressed in **game ECEF**: origin at Earth’s center; **+Y = North Pole**; prime meridian (0° longitude) lies in the **+X / −Z** half-plane.
- **Geodetic ↔ ECEF** conversions use **WGS84** (`constants.rs`, `earth.rs`).
- The integrator advances state in the **rotating Earth frame**. Non‑inertial accelerations are applied explicitly:
  - **Coriolis:** \(-2\,\boldsymbol{\Omega} \times \mathbf{v}\)
  - **Centrifugal:** \(-\,\boldsymbol{\Omega} \times (\boldsymbol{\Omega} \times \mathbf{r})\)
  with \(\boldsymbol{\Omega}\) along **+Y** (sidereal rate).

## Gravity

- **Central field:** \(\mathbf{a}_0 = -\mu\,\mathbf{r}/|\mathbf{r}|^3\) with \(\mu = G M_\oplus\) (`GRAVITY_CONSTANT`).
- **Oblateness / zonal harmonics:** J₂, J₃, J₄ (EGM96‑style dimensionless coefficients; equatorial radius `WGS84_A` in the J₂–J₄ terms). Spherical‑harmonic formulas use a conventional **Z‑north** ECEF internally, then map back to game ECEF (`gravity.rs`).
- **Not modeled:** tesseral harmonics, Sun/Moon, tides, relativistic corrections.

## Atmosphere

- **0–86 km (geometric):** **US Standard Atmosphere 1976** layers (lapse and isothermal segments; hydrostatic pressure with `G0` and dry‑air `R_AIR`) — `atmosphere.rs`.
- **> 86 km:** **Thermosphere extension:** piecewise log‑linear density and linear temperature between altitude knots (rough **MSIS‑class** means for mid‑latitude); exponential decay beyond the last knot. This is **not** NRLMSISE‑00 or JB2008 (no solar / geomagnetic drivers).

From density \(\rho\) and temperature \(T\): **ideal gas** \(p = \rho R T\), **speed of sound** \(a = \sqrt{\gamma R T}\) (\(\gamma = 1.4\)).

## Aerodynamics

- **Drag:** \(\mathbf{F}_d = -\tfrac{1}{2}\rho |\mathbf{v}|^2 C_d(M) A\) with **empirical** \(C_d(\mathrm{Mach})\) (`aerodynamics.rs`). No lift, sideslip, or attitude dynamics unless extended elsewhere.

## Propulsion

- **Mass flow:** normalized **solid‑motor‑like** thrust profile \(w(\tau)\) over each stage burn; ṁ scaled so total expelled mass matches stage `fuel_mass` (`propulsion.rs`).
- **Thrust magnitude:** vacuum momentum term \(\dot m\,I_{sp,vac}\,g_0\) minus a **linear pressure correction** from `Isp_sl` and ambient pressure (standard simplified nozzle model), not full \(C_F(p_e/p_a)\) tables.

## Vehicle / flight logic

- **Boost steering** (loft / pitch schedule) and **flight phases** (boost / ballistic / re‑entry) live in `missiles.rs` (`BallisticMissilePhysics`), not in `physics/`.

## Numerical integration

- State \((\mathbf{r}, \mathbf{v}, m)\) is advanced with a **Dormand–Prince DOPRI5** step (5th-order formula, **6 RHS evaluations** per step): `physics/integration.rs`. This is the usual **RK45 / ode45** family (embedded 4th order exists in the literature for error control; the game uses **fixed** step size `dt` / `sub_dt`, not adaptive step doubling).
- **Stage jettison** (discrete dry-mass drops at stage boundaries) is applied **after** each accepted RK step, not inside the ODE RHS.
- Re-entry bodies and impact prediction use the same **DOPRI5** kernel on \((\mathbf{r}, \mathbf{v})\) with constant mass.
- Optional **substeps** near the ground (low altitude + high speed) remain in `physics_system` to limit tunneling through the ellipsoid.

For implementation entry points, see module documentation in `src/physics/mod.rs`.
