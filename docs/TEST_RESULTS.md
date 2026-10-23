# Torque Lab Native — Automated Physics & Behavioral Validation Report (`TEST_RESULTS.md`)

**Project:** Torque Lab Native (C++20 / OpenGL 3.3 Core / GLSL 330 / GLFW / GLAD / Bullet Physics / Dear ImGui)  
**Test Binary:** `build/Release/TorqueLabNativeTests.exe`  
**Desktop App Smoke Test:** `build/Release/TorqueLabNative.exe --smoke-test`  
**Web Baseline Reference:** `npm.cmd test` (`vitest run` with `@dimforge/rapier3d-compat`)  
**Verification Status:** **18 / 18 Native Test Suites Passed (`0` failed assertions)** + **18 / 18 Web Baseline Test Suites Passed**

---

## 1. Executive Summary

Both **Torque Lab Native** (`TorqueLabNativeTests.exe` and `TorqueLabNative.exe`) and the source **Torque Lab Web** (`npm.cmd test`) were compiled and executed on Windows x64 (Visual Studio 18 2026 / MSVC 19.50).

- **Native Unit & Validation Test Suite (`TorqueLabNativeTests.exe`):** Passed **18 / 18 suites** (`Unit 1–8` and `Tests A–J`) with **0 failed assertions** in `< 100 ms` total wall-clock time.
- **Native OpenGL 3.3 + Bullet + Dear ImGui Smoke Test (`TorqueLabNative.exe --smoke-test`):** Initialized GLFW, OpenGL 3.3 Core context, GLAD function loader, Dear ImGui HUD, compiled all 3 GLSL 330 shader programs (`shadow_depth`, `phong_shadow`, `debug_line`), allocated the `2048×2048` depth shadow-map FBO, ran the built-in physics self-check (`Eq Load Err: 0.00% | Accel Squat Pitch: +0.79 deg @ 28 km/h | Brake Dive Pitch: -1.68 deg`), and rendered 15 full 3D + HUD frames with exit code `0`.

---

## 2. Unit Test Suite Results (`Unit 1` – `Unit 8`)

| Suite ID | Subsystem | Native C++ Result | Verified Equations & Physical Invariants |
| :--- | :--- | :---: | :--- |
| **Unit 1** | `SuspensionSystem` (`src/vehicle/SuspensionSystem.cpp`) | **PASS** | Linear spring $F_{\text{spring}} = -kx$, asymmetric bound/rebound damper $F_{\text{damper}} = -cv$, progressive quadratic+linear bump stop below `suspensionMinLength`, and axle anti-roll bar $F_{\text{arb}} = k_{\text{arb}}(c_L - c_R)$ with zero net vertical force creation ($F_L + F_R = 0$). |
| **Unit 2** | `Math3D` (`src/physics/Math3D.h`) | **PASS** | Right-handed vehicle basis (`+X` Left, `+Y` Up, `+Z` Forward; $\hat{x} \times \hat{y} = \hat{z}$), `localPointToWorld` / `worldPointToLocal` exact round-trip, pitch extraction ($\theta > 0$ nose up), and roll extraction ($\phi > 0$ right body roll). |
| **Unit 3** | `TireModel` (`src/vehicle/TireModel.cpp`) | **PASS** | Load-sensitive friction $\mu_{\text{eff}}(F_z)$ decreases monotonically with increasing $F_z$ ($\frac{F_{\max}(7400\text{ N})}{F_{\max}(3700\text{ N})} < 2.0$), and combined longitudinal/lateral slip strictly obeys the **Friction Circle** $\sqrt{F_x^2 + F_y^2} \le \mu_{\text{eff}} F_z$. |
| **Unit 4** | `TireModel` Kinematics (`src/vehicle/TireModel.cpp`) | **PASS** | Free-rolling wheel ($\omega = v_x / r_w$) produces $\kappa = 0$; driven wheel ($\omega r_w > v_x$) produces $\kappa > 0, F_x > 0$; braked wheel ($\omega r_w < v_x$) produces $\kappa < 0, F_x < 0$; leftward drift ($v_y > 0$) produces $\alpha < 0$ and restoring rightward force $F_y < 0$. |
| **Unit 5** | `Drivetrain` (`src/vehicle/Drivetrain.cpp`) | **PASS** | Piecewise-linear torque curve interpolation (peak $405\text{ N}\cdot\text{m}$ @ $4500\text{ RPM}$), gear ratio ordering ($|G_1| > |G_2| > \dots > |G_6|$, $G_R < 0, G_N = 0$), fuel-cut rev limiter at `maxRpm = 7000 RPM`, and viscous LSD torque transfer from faster-spinning to slower-spinning wheel. |
| **Unit 6** | `SurfaceSystem` (`src/physics/SurfaceSystem.cpp`) | **PASS** | Strict monotonic friction hierarchy across all 6 surfaces: $\mu_{\text{dry}} (1.00) > \mu_{\text{rumble}} (0.88) > \mu_{\text{wet}} (0.65) > \mu_{\text{grass}} (0.55) > \mu_{\text{gravel}} (0.48) > \mu_{\text{ice}} (0.20)$. |
| **Unit 7** | `SteeringSystem` (`src/vehicle/SteeringSystem.cpp`) | **PASS** | Left turn ($\delta > 0$) steers inner Front-Left wheel more than outer Front-Right wheel ($\delta_{FL} > \delta_{FR} > 0$); right turn ($\delta < 0$) mirrors symmetrically ($|\delta_{FR}| > |\delta_{FL}|$); speed-sensitive limit decreases monotonically with $|v_{\text{fwd}}|$. |
| **Unit 8** | `Aerodynamics` & `BrakeSystem` (`src/vehicle/Aerodynamics.cpp`, `BrakeSystem.cpp`) | **PASS** | Doubling airspeed ($20 \to 40\text{ m/s}$) quadruples aerodynamic drag and downforce ($4.000\times$); service brake distributes torque by `brakeFrontBias = 0.64`; handbrake applies torque exclusively to rear wheels. |

---

## 3. Full-Simulation Validation Scenarios (`Test A` – `Test J`)

| Scenario | Description | Native C++ Metrics (`TorqueLabNativeTests.exe`) | Acceptance Threshold | Status |
| :--- | :--- | :--- | :--- | :---: |
| **Test A** | **Static Equilibrium on Flat Ground** (3.0 s settle from $y = 0.60\text{ m}$) | $\sum F_z = 14,715.0\text{ N}$ (`Err = 0.000%`), `RideHeight = 0.4910 m`, `FL_Comp = 0.1093 m`, $|v_y| < 0.0001\text{ m/s}$ | $\left|\sum F_z - mg\right| / mg < 2\%$, $c_i \in [0.08, 0.16]\text{ m}$, $|v_y| < 0.005\text{ m/s}$ | **PASS** |
| **Test B** | **Acceleration & Rear Squat** (2.0 s full throttle on Dry Asphalt) | $v_{\text{fwd}} = 10.54\text{ m/s}$ ($37.9\text{ km/h}$), $a_{\text{long}} = 5.57\text{ m/s}^2$, $\theta = +0.804^\circ$ ($+0.0140\text{ rad}$), $\Delta F_{z,\text{rear}} = +1,328\text{ N}$ | $v_{\text{fwd}} > 10.0\text{ m/s}$, $a_{\text{long}} > 2.5\text{ m/s}^2$, $\theta > +0.004\text{ rad}$, $\Delta F_{z,\text{rear}} > +300\text{ N}$ | **PASS** |
| **Test C** | **Service Braking & Front Dive** ($85\%$ brake from $22\text{ m/s}$, sampled at $0.5\text{ s}$) | $v_{\text{fwd}} = 16.95\text{ m/s}$, $a_{\text{long}} = -9.82\text{ m/s}^2$, $\theta = -1.610^\circ$ ($-0.0281\text{ rad}$), $F_{z,\text{front}} = 9,970\text{ N}$ vs. $F_{z,\text{rear}} = 4,704\text{ N}$ | $a_{\text{long}} < -5.0\text{ m/s}^2$, $\theta < -0.008\text{ rad}$, $F_{z,\text{front}} > F_{z,\text{rear}} + 1,500\text{ N}$ | **PASS** |
| **Test D** | **Cornering & Reversible Body Roll** (Left turn then Right turn at $18\text{ m/s}$) | Left Turn: $\phi = +2.30^\circ$ ($a_{\text{lat}} = +9.70\text{ m/s}^2$)<br>Right Turn: $\phi = -2.27^\circ$ ($a_{\text{lat}} = -9.57\text{ m/s}^2$) | Left: $\phi > +0.01\text{ rad}, F_{z,R} > F_{z,L} + 1,200\text{ N}$<br>Right: $\phi < -0.01\text{ rad}, F_{z,L} > F_{z,R} + 1,200\text{ N}$ | **PASS** |
| **Test E** | **Low-Grip Surface Effects** (Dry Asphalt $\mu=1.0$ vs. Ice $\mu=0.20$) | **Dry:** $\bar{\kappa} = 0.06$, $v_{\text{launch}} = 6.40\text{ m/s}$, $d_{\text{stop}} = 11.36\text{ m}$<br>**Ice:** $\bar{\kappa} = 12.91$, $v_{\text{launch}} = 1.24\text{ m/s}$, $d_{\text{stop}} = 51.34\text{ m}$ | $\bar{\kappa}_{\text{ice}} > 1.5\bar{\kappa}_{\text{dry}}$, $v_{\text{ice}} < 0.55 v_{\text{dry}}$, $d_{\text{stop,ice}} > 2.5 d_{\text{stop,dry}}$ | **PASS** |
| **Test F** | **Uneven Terrain (Speed Bump Response & Damping)** ($12\text{ m/s}$ over $80\text{ mm}$ bump) | $c_{\text{eq}} = 0.109\text{ m} \to c_{\text{front,peak}} = 0.206\text{ m}, c_{\text{rear,peak}} = 0.208\text{ m}$, $\theta_{\text{peak}} = 2.87^\circ \to \theta_{\text{final}} = 0.08^\circ$ | $c_{\text{peak}} > c_{\text{eq}} + 0.035\text{ m}$, $\theta_{\text{peak}} > 0.015\text{ rad}$, $|\theta_{\text{final}}| < 0.01\text{ rad}$ | **PASS** |
| **Test G** | **Parameter Sensitivity** ($1.6\times$ Spring Stiffness & CoM Height $-0.10\text{ m}$ vs. $+0.28\text{ m}$) | Compression: $0.1093\text{ m} \to 0.0683\text{ m}$ ($-37.5\%$)<br>Body Roll: $1.76^\circ$ (Low CoM) vs. $3.60^\circ$ (High CoM, $+104.5\%$) | $c_{\text{stiff}} < 0.72 c_{\text{base}}$, $\phi_{\text{highCoM}} > 1.25 \phi_{\text{lowCoM}}$ | **PASS** |
| **Test H** | **Collision Momentum Transfer & Reset** ($15\text{ m/s}$ impact into wall at $z = 14\text{ m}$) | Post-crash $z = 11.12\text{ m}$, $v_{\text{fwd}} = -0.81\text{ m/s}$ (rebound), `Sanity = OK`; `resetVehicle()` restores exact $(0, y_{\text{eq}}, 0)$ and $\mathbf{v} = \mathbf{0}$ | $z < 13.8\text{ m}$ (no tunneling), $v_{\text{fwd}} < 2.0\text{ m/s}$, clean reset | **PASS** |
| **Test I** | **Time-Step Independence** ($30\text{ FPS}$, $60\text{ FPS}$, $120\text{ FPS}$ accumulator updates) | `30 FPS`: $z = 9.427\text{ m}, v = 9.935\text{ m/s}$<br>`60 FPS`: $z = 9.427\text{ m}, v = 9.935\text{ m/s}$<br>`120 FPS`: $z = 9.427\text{ m}, v = 9.935\text{ m/s}$ | $|\Delta z| < 0.25\text{ m}$, $|\Delta v| < 0.20\text{ m/s}$ across frame rates | **PASS** |
| **Test J** | **Coasting Energy & Numerical Sanity** (Neutral coast from $25.00\text{ m/s}$ for $3.0\text{ s}$) | $v(0) = 25.00\text{ m/s} \to v(3\text{ s}) = 24.13\text{ m/s}$ (monotonically decreasing), `Sanity = OK` | $v(t)$ strictly non-increasing, $v(3\text{ s}) \in [23.8, 24.4]\text{ m/s}$ | **PASS** |

---

## 4. Documented Engine-Level Engineering Solutions (Rapier 3D vs. Bullet Physics)

During the migration from Rapier 3D (`@dimforge/rapier3d-compat`) to native Bullet Physics (`btDiscreteDynamicsWorld`), two engine-level architectural differences were identified and solved to achieve $< 0.01\%$ static equilibrium parity and 100% validation passage:

1. **Analytical 64-Bit OBB Slab Raycasting vs. Bullet 32-Bit GJK Raycasting on Large Ground Boxes:**
   - **Root Cause:** Bullet's `btCollisionWorld::rayTestSingleInternal` treats `btBoxShape` as a generic `btConvexShape` and runs iterative 32-bit float GJK (`btSubsimplexConvexCast`) against the 8 corners of the box. On a $1000\text{ m} \times 1\text{ m} \times 1000\text{ m}$ ground box (`halfExtents = (500, 0.5, 500)`), Voronoi simplex cross-products on $500\text{ m}$ vectors in 32-bit `float` suffer from catastrophic cancellation, skewing the hit normal (`normal.y ≈ 0.81` instead of `1.00`) and stopping the ray ~`18 mm` early.
   - **Solution Implemented in `src/vehicle/VehicleDynamics.cpp`:** Implemented `castRayExactWorld()`, an exact 64-bit `double` Kay-Kajiya Oriented Bounding Box (OBB) slab intersection test against all `BOX_SHAPE_PROXYTYPE` colliders in the Bullet world (matching Rapier's analytical `Cuboid::cast_ray_and_get_normal`), with automatic fallback to Bullet's `m_world->rayTest` for any non-box collision shapes. This reduced static equilibrium vertical load error from `19.2%` to **`0.000%` (`14,715.0 N` vs `14,715.0 N`)**.
2. **Dynamic Center-of-Mass (CoM) Frame Translation (`VehicleBody`):**
   - **Root Cause:** Rapier allows a rigid body's transform origin to stay at the geometric chassis origin while offsetting the Center of Mass via `setAdditionalMassProperties`. In Bullet (`btRigidBody`), the world transform origin **is** the Center of Mass.
   - **Solution Implemented in `src/vehicle/VehicleBody.cpp`:** `VehicleBody` shifts the child `btBoxShape` transforms inside `btCompoundShape` by $-\mathbf{r}_{\text{CoM}}$ and translates transparently between the geometric chassis origin $\mathbf{p}_{\text{origin}}$ and Bullet's CoM origin $\mathbf{p}_{\text{CoM}} = \mathbf{p}_{\text{origin}} + \mathbf{R}\mathbf{r}_{\text{CoM}}$, including live runtime updates when the `CoM Height` or `Weight Distribution` sliders are adjusted in Dear ImGui.
