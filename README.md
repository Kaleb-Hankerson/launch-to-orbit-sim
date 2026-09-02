# launch-to-orbit-sim

A 2D Python simulation of a staged rocket's ascent from launch through orbital insertion (or, honestly, near-miss suborbital reentry) that includes thrust, atmospheric drag, gravity, RK4 integration, a flight-phase state machine, and orbital mechanics (vis-viva, eccentricity, period).

A companion C++ port of the core physics engine exists at [launch-physics-cpp](https://github.com/Kaleb-Hankerson/launch-physics-cpp).

## What it does

- Simulates a two-stage rocket ascending through the atmosphere, coasting, and either reaching orbit or falling back to Earth
- Models thrust, atmospheric drag (exponential density model), gravity, and a linear pitch program
- Uses RK4 (4th-order Runge-Kutta) integration
- Tracks flight phase via a state machine (`PRE_LAUNCH` → `POWERED_FLIGHT` → `COAST` → `ORBIT_INSERTION`)
- Computes dynamic pressure and tracks Max Q
- Computes orbital elements (semi-major axis, eccentricity, period) via the vis-viva equation once orbit is detected
- Logs full telemetry to CSV, with a standalone script (`read_telemetry.py`) that reads it back independently
- Plots trajectory, velocity components, and total speed, plus an animated, phase-colored replay of the trajectory
- Includes a unit test validating the RK4 integrator against the analytical projectile-motion formula

## Running it

```
python main.py
```


Produces `telemetry.csv`, prints final flight phase/eccentricity/period, and shows the trajectory, velocity, speed, and animated plots.

To run the unit test:

```
python -m unittest test_rocket.py
```

## Design

- **RK4 integration**, adopted after starting with Euler (used in an early practice script). Euler's per-step error accumulates meaningfully over a long simulation; empirically found ~2-3m of drift over just a 5-second, 50-step ballistic test at dt=0.1s.
- **Pure derivative function.** `calc_derivatives()` takes any position/velocity/mass/time, real or hypothetical, and returns rates of change, with no side effects. This lets RK4 safely sample it at the k1-k4 midpoint/endpoint states. The actual state commit happens once per real step, in `rk4_stepper()`.
- **3DOF (point-mass) model**, not 6DOF, so no rocket orientation or rotational dynamics. A deliberate scope choice; see Limitations.
- **Distance-from-Earth's-center**, used for orbital mechanics, is computed as a full vector magnitude (`[x, earth_radius + y]`), not just altitude plus Earth's radius. The simpler version breaks down once downrange distance becomes comparable to Earth's radius, which happens well before apogee here.

## Validation

The RK4 integrator was tested in isolation (zero thrust, zero drag, known starting velocity) against the analytical projectile-motion formula and it passed within a 3m tolerance over a 5-second, 50-step simulation, consistent with Euler's known ~2-3m accumulated error at the same settings.

## Comparison to a Real Launch Vehicle

Not an attempt to replicate a specific mission, this rocket is fictional and smaller-scale. The comparison below checks that the simulation's core quantities (thrust-to-weight, burn timescale) land in a physically realistic range for an orbital-class vehicle.

| Metric                          | This Sim (Staged)   | Falcon 9 Full Thrust     |
|----------------------------------|----------------------|----------------------------|
| Liftoff mass                    | 47,500 kg            | 549,000 kg                |
| Liftoff thrust-to-weight ratio  | ~1.61                | ~1.41                     |
| Stage 1 burn time               | 152 s                | 162 s                     |
| Stage 1 exhaust velocity        | 3,000 m/s            | ~2,770 m/s (sea level)    |
| Velocity needed for LEO         | ~7,800 m/s           | ~7,800 m/s (achieved)     |
| Velocity achieved at cutoff     | ~11,800 m/s (suborbital, see Limitations) | ~7,800+ m/s (achieved) |

Sources: [Falcon Payload User's Guide, SpaceX (2025)](https://www.spacex.com/assets/media/falcon-users-guide-2025-05-09.pdf) (thrust); [Falcon 9 Full Thrust, Wikipedia](https://en.wikipedia.org/wiki/Falcon_9_Full_Thrust) (mass, burn time)

## Known limitations (deliberate simplifications)

- Constant thrust, exhaust velocity, and mass flow rate per stage(no throttle profile)
- Constant drag coefficient that doesn't vary with Mach number (no transonic spike modeled)
- 3DOF (point-mass) model(no rocket orientation or rotational dynamics)
- Simplified exponential atmospheric density model, not a full standard atmosphere table
- No wind modeling (negligible effect on trajectory at this thrust scale for a point-mass model). Wind's real operational significance (structural loading from wind shear) is outside any point-mass model's scope, 3DOF or 6DOF
- The `ORBIT_INSERTION` flight-phase trigger only checks that the vis-viva-derived semi-major axis is positive (elliptical vs. hyperbolic). It does not verify periapsis clears Earth's surface. As a result, this rocket's `flight_phase` reports `ORBIT_INSERTION` briefly right as it falls back to Earth on reentry (visible as a brief green segment at the very end of the animated plot).
- **The rocket does not reliably reach a genuinely stable orbit.** This was investigated from three independent angles: adding a second stage (nearly doubled peak speed, 5,600 → 11,800 m/s, but eccentricity stayed extreme at 0.9998); replacing the linear pitch program with a velocity-following gravity turn (caused a genuine steering instability where the angle could swing past 90°, pushing thrust backward and reversing the trajectory; a monotonic-decrease clamp fixed the instability but not the underlying suborbital result); and retuning the linear pitch program's shape and duration (no meaningful improvement, eccentricity 0.9999). All three point to the same conclusion: neither more delta-v nor better-tuned reactive/scheduled steering can reliably produce genuinely tangential cutoff velocity. That likely requires real closed-loop guidance (e.g. Linear Tangent Guidance), which is deliberately out of scope for this project.

## Notable behaviors

- The simulation shows an emergent, sharp deceleration event on reentry (both velocity components drop rapidly) as the rocket falls back through denser lower atmosphere at high speed. This wasn't explicitly programmed, it falls directly out of the exponential density model combined with the v² drag term.
- After switching from Euler to RK4, the coast-phase trajectory showed a very slight curve that hadn't been visible under Euler. RK4's higher accuracy resolves a nonzero (though tiny) residual drag that Euler's larger per-step error had been masking.

## Files

- `rocket.py` — the physics engine (`Rocket` class, `FlightPhase` state machine)
- `main.py` — runs the simulation, writes telemetry, produces plots
- `read_telemetry.py` — standalone script demonstrating the CSV can be analyzed independently of the simulation
- `test_rocket.py` — unit test validating RK4 against analytical projectile motion

## Companion project

See [launch-physics-cpp](https://github.com/Kaleb-Hankerson/launch-physics-cpp) for a C++ port of the core physics engine and RK4 integrator, written with embedded-style coding discipline (no dynamic allocation, no exceptions, `double` precision throughout) targeting the embedded/GN&C side of the aerospace software field.