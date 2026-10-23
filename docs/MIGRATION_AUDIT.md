# Torque Lab — Source Application Migration Audit (`MIGRATION_AUDIT.md`)

**Project:** Torque Lab Native (C++20 / OpenGL 3.3+ / Bullet Physics / Dear ImGui)  
**Source Audited:** Browser-based Torque Lab (`TypeScript + Three.js + @dimforge/rapier3d-compat`)  
**Audit Status:** Complete — 100% of source files inspected and verified against automated tests.

---

## 1. Executive Summary

The existing **Torque Lab** browser application is a complete, self-contained 6-DOF vehicle dynamics simulator written in TypeScript using **Three.js** for 3D rendering and **Rapier 3D (`@dimforge/rapier3d-compat`)** for rigid-body dynamics, raycasting, and collision detection, paired with a **custom-engineered vehicle physics layer** in `src/physics/`.

Every physical behavior—pitch during braking/acceleration, body roll during cornering, tire friction-circle saturation, load-sensitive grip, wheel angular momentum integration, Ackermann steering, engine torque curves, rev limiting, gear shifting, viscous limited-slip differential, aerodynamic drag/downforce, and surface-dependent traction—is **fully implemented** and verified by 18 automated unit and integration tests (`tests/unit.test.ts` and `tests/validation.test.ts`).

---

## 2. Source File Inventory & Implementation Status

| Source File | Lines | Subsystem | Status | Key Responsibilities |
| :--- | :---: | :--- | :---: | :--- |
| `src/physics/math.ts` | 309 | Core Math | **Fully Implemented** | `Vec3`, `Quat`, right-handed basis extraction (`+X` Left, `+Y` Up, `+Z` Forward), quaternion slerp, local/world point & vector transforms, pitch/roll/yaw Euler extraction. |
| `src/physics/VehicleConfig.ts` | 191 | Configuration | **Fully Implemented** | Complete SI parameter specification (`mass`, `comHeightOffset`, `frontWeightDistribution`, `inertiaTensor`, geometry, springs/dampers, Pacejka tire constants, Ackermann steering, 6-speed + R drivetrain, brakes, aero), plus `getWheelMountPositionsLocal`, `getCenterOfMassLocal`, and `computeEquilibriumChassisHeight`. |
| `src/physics/SurfaceSystem.ts` | 115 | Road Surfaces | **Fully Implemented** | Catalog of 6 surface types (`dry_asphalt`, `wet_asphalt`, `gravel`, `ice`, `grass`, `rumble_strip`) with friction, cornering stiffness, and rolling resistance multipliers; maps collider handles to surface properties. |
| `src/physics/WheelState.ts` | 119 | Wheel State | **Fully Implemented** | Per-wheel state structure (`FL`, `FR`, `RL`, `RR`) storing raycast contact, suspension compression/velocity/forces, slip ratio $\kappa$, slip angle $\alpha$, tire forces $F_x, F_y, F_z$, angular velocity $\omega$, spin angle, and torques. |
| `src/physics/SuspensionSystem.ts` | 137 | Suspension | **Fully Implemented** | Linear spring + asymmetric compression/rebound damper ($F_s = -kx - cv$), progressive quadratic bump stops near `minLength`, and front/rear axle anti-roll bars ($F_{\text{arb}} = k_{\text{arb}}(c_L - c_R)$). |
| `src/physics/TireModel.ts` | 212 | Tire Dynamics | **Fully Implemented** | Load-sensitive friction coefficient $\mu_{\text{eff}}(F_z)$, longitudinal slip ratio $\kappa$, lateral slip angle $\alpha$, Pacejka-style single-peak/saturation curves, and combined-slip **Friction Circle** $\sqrt{F_x^2 + F_y^2} \le \mu_{\text{eff}} F_z$. |
| `src/physics/SteeringSystem.ts` | 88 | Steering | **Fully Implemented** | Rate-limited steering input response, speed-sensitive maximum steer angle $\delta_{\max}(v) = \frac{\delta_0}{1 + k_v |v_{\text{fwd}}|}$, and Ackermann inner/outer front wheel geometry. |
| `src/physics/Drivetrain.ts` | 269 | Powertrain | **Fully Implemented** | 9-point engine torque curve (`800–7300 RPM`), rev limiter (`7000 RPM`), closed-throttle engine braking, automatic (`6400/2400 RPM` hysteresis) & manual (`R, N, 1–6`) gearboxes with `0.14 s` shift torque cut, launch clutch slip below `1450 RPM`, `RWD`/`FWD`/`AWD` torque split, and viscous LSD locking torque. |
| `src/physics/BrakeSystem.ts` | 48 | Brakes | **Fully Implemented** | Front-biased (`64% F / 36% R`) service brakes on all 4 wheels and mechanical rear-axle handbrake (`2600 N·m`). |
| `src/physics/Aerodynamics.ts` | 91 | Aerodynamics | **Fully Implemented** | Quadratic air drag $\mathbf{F}_{\text{drag}} = -\frac{1}{2}\rho C_d A |\mathbf{v}|\mathbf{v}$, speed-squared aerodynamic downforce, and speed-smoothed rolling resistance torque. |
| `src/physics/VehicleBody.ts` | 246 | Rigid Body | **Fully Implemented** | Rapier dynamic `RigidBody` wrapper with compound lower-tub + upper-cabin `Cuboid` colliders, runtime `syncMassProperties` (mass, local CoM, inertia tensor), off-center force application `addForceAtPoint`, finite-difference acceleration estimation, and active collision contact extraction. |
| `src/physics/VehicleDynamics.ts` | 538 | Orchestrator | **Fully Implemented** | Complete 120 Hz step integration (`step(dt)`), 4-wheel raycast suspension + ARB + tire forces + 8-substep rotational wheel integration ($\alpha_w = (T_{\text{drive}} - F_x r_w - T_{\text{brake}}\text{sgn}(\omega))/I_w$), low-speed longitudinal damping, auto-reverse convenience logic, render transform interpolation, and `VehicleTelemetrySnapshot`. |
| `src/physics/PhysicsWorld.ts` | 350 | World & Track | **Fully Implemented** | Fixed `1/120 s` accumulator loop (`maxSubSteps = 12`), Proving Ground track builder (main straight, wet skidpad, gravel lane, ice strip, 6 suspension bumps, 2 launch/landing ramps, slalom bollards, perimeter crash barriers), and 7 teleport presets (`1`–`7`). |
| `src/rendering/VehicleRenderer.ts` | 381 | Car Rendering | **Fully Implemented** | Multi-part procedural sports coupe mesh (painted body, sloped hood, cabin, tinted glass, splitter, grille, side skirts, diffuser, rear spoiler, headlights, dynamic brake & reverse lights, interior steering wheel with 14:1 rotation) + 4 independent wheel/tire/5-spoke rim/ventilated rotor groups, non-spinning steered brake caliper knuckles, and scaling yellow coilover struts. |
| `src/rendering/TrackRenderer.ts` | 257 | Track Rendering | **Fully Implemented** | Procedural 256×256 grid/noise/stripe textures for all 6 surface types, directional sun shadow map following the car, hemisphere fill light, exponential fog, start line, dashed center line, 25 m distance posts, and North Handling Circle ring (`R = 28 m`). |
| `src/rendering/DebugOverlay3D.ts` | 330 | 3D Debug Gizmos | **Fully Implemented** | World-space 3D telemetry vectors: yellow CoM sphere, forward/lateral basis arrows, velocity & acceleration arrows, suspension ray lines, contact spheres (green/red skidding), normal load $F_z$ arrows, horizontal tire force $\mathbf{F}_{xy}$ arrows, dynamic ground friction-circle rings ($\mu_{\text{eff}}F_z$), and pink collision contact spheres. |
| `src/rendering/CameraController.ts` | 183 | Camera System | **Fully Implemented** | 4 camera modes (`Chase Follow`, `Orbit Inspection` with mouse drag + scroll zoom, `Wheel & Suspension Rig`, `Driver Hood` attached to chassis pitch/roll). |
| `src/ui/InputController.ts` | 197 | Input System | **Fully Implemented** | Progressive keyboard pedal ramping (`W/S/A/D/Arrows/Space`), shortcut keys (`R`, `C`, `P`, `T`, `G`, `M`, `E`, `Q`, `Escape`, `1`–`7`), and optional Gamepad API polling. |
| `src/ui/DashboardUI.ts` | 303 | HUD & Cluster | **Fully Implemented** | Top bar with 7 zone teleport buttons, action toggles, quality selector (`High/Medium/Low`),Verify Physics self-test toast, left Driving Controls card, and bottom Instrument Cluster (speed km/h & m/s, gear & AUTO/MAN badge, RPM bar with redline, THR/BRK bars, steering angle bar, 2D G-G diagram dot, Pitch/Roll readouts, and active surface pill). |
| `src/ui/TelemetryPanel.ts` | 253 | Telemetry & Plots | **Fully Implemented** | Performance summary (`FPS`, `Phys Step ms`, `Aero Drag`, `Downforce`, `Contacts`, `Sanity`), 4-wheel live table (`Comp mm/%`, `Fz/Grip`, `Slip κ/α`, `Fx/Fy`, `ω`), and 4-band real-time oscilloscope (`Speed`, `Long/Lat Accel`, `Pitch/Roll`, `4-Wheel Suspension Travel`). |
| `src/ui/TuningPanel.ts` | 313 | Live Tuning | **Fully Implemented** | Drivetrain selector (`RWD`, `FWD`, `AWD`), 12 live sliders (`mass`, `comHeight`, `weightDist`, `springStiff`, `suspDamping`, `antiRoll`, `tireMu`, `tireLatStiff`, `steerLimit`, `brakeStrength`, `engineMult`, `dragCoeff`), and `Defaults` reset button. |
| `tests/unit.test.ts` | 312 | Unit Tests | **Fully Implemented** | 8 unit test suites covering suspension signs, coordinate transforms, friction circle & load sensitivity, wheel slip, drivetrain/rev-limiter, surface friction hierarchy, Ackermann steering, and quadratic aero/brakes. |
| `tests/validation.test.ts` | 440 | Validation Tests | **Fully Implemented** | 10 full-simulation physical validation scenarios (`Tests A–J`): static equilibrium, acceleration squat, braking dive, cornering roll reversal, dry vs. ice launch/braking, speed bump damping, parameter sensitivity, wall collision & reset, timestep independence (`30/60/120 Hz`), and coasting energy sanity. |

---

## 3. Detailed Subsystem Findings & Migration Implications

### 3.1 Rigid-Body Center of Mass (CoM) Offset Between Rapier and Bullet
- **Web Implementation (`src/physics/VehicleBody.ts`):** Uses Rapier's `rigidBody.setAdditionalMassProperties(mass, comLocal, principalAngularInertia, quatIdentity())`, which allows the rigid body's transform origin (`rigidBody.translation()`) to remain at the **geometric chassis origin** while the physical Center of Mass is offset by $\mathbf{r}_{\text{CoM}} = (0, \text{comHeightOffset}, (\text{frontWeightDistribution} - 0.5) \cdot L)$.
- **Native Bullet Physics Requirement:** In Bullet (`btRigidBody`), the rigid body's world transform origin **is** its Center of Mass.
- **Migration Solution:** Our native `VehicleBody` class wraps a `btCompoundShape` whose child `btBoxShape` colliders are shifted by $-\mathbf{r}_{\text{CoM}}$ relative to the Bullet CoM frame, and provides exact conversion methods between the **Geometric Chassis Origin** $\mathbf{p}_{\text{origin}}$ (used by suspension mounts, raycasts, telemetry, and rendering) and the **Bullet CoM World Transform** $\mathbf{p}_{\text{CoM}} = \mathbf{p}_{\text{origin}} + \mathbf{R}\mathbf{r}_{\text{CoM}}$. When runtime tuning sliders change `mass`, `comHeightOffset`, or `frontWeightDistribution`, `VehicleBody::syncMassProperties()` updates the child transforms in the `btCompoundShape`, calls `body->setMassProps(mass, inertia)`, and preserves the world-space geometric chassis pose without discontinuity.

### 3.2 Tire Force Application Lever Arm
- **Web Implementation (`src/physics/VehicleDynamics.ts`):** Applies horizontal tire forces at a blended height between the ground contact point and the wheel hub (`tireForceApplicationHeightOffset = 0.06 m` above the contact point) to model suspension anti-dive/anti-squat/roll-center geometry so that longitudinal and lateral forces produce realistic, non-exaggerated pitch and roll moments around the CoM.
- **Migration Solution:** Preserve the exact same force application point computation in C++ (`forceAppPoint = contactPointWorld + contactNormalWorld * config.tireForceApplicationHeightOffset`) and apply via Bullet's `btRigidBody::applyForce(totalForce, forceAppPoint - comWorld)`.

### 3.3 Sub-stepped Wheel Rotational Integration
- **Web Implementation (`src/physics/VehicleDynamics.ts`):** Integrates wheel angular velocity $\omega$ using `wheelSubSteps = 8` per `1/120 s` physics step (`subDt = dt / 8 = 1/960 s`), recomputing slip ratio $\kappa(\omega)$ and longitudinal force $F_x(\kappa)$ at each substep and clamping zero-crossings caused by passive braking torque.
- **Migration Solution:** Port this exact 8-substep loop verbatim into C++ so high-stiffness Pacejka slip integration at low speeds is 100% stable and numerically identical.

### 3.4 Visual Rendering & UI Parity
- **Web Implementation:** Uses Three.js `MeshStandardMaterial` with directional shadow mapping (`2048x2048` High, `1024x1024` Medium, disabled Low), hemisphere ambient lighting, exponential fog, procedural 256×256 surface textures, and HTML/Canvas HUD panels.
- **Migration Solution:**
  - Native OpenGL 3.3 Core renderer with custom GLSL shaders (`shaders/phong_shadow.vert/.frag`, `shaders/shadow_depth.vert/.frag`, `shaders/debug_line.vert/.frag`) implementing hemisphere sky/ground ambient lighting, Blinn-Phong + Fresnel metallic/roughness shading, `2048x2048` PCF directional shadow mapping following the vehicle, exponential depth fog, and procedural 256×256 RGB textures generated in C++ CPU memory and uploaded with mipmaps + repeat wrapping.
  - Native **Dear ImGui** overlay replicating all 4 UI regions: Top Control & Proving Ground Zone Bar (`1`–`7`), Left Driving Controls Reference Card, Right Telemetry & 4-Band Real-Time Oscilloscope Panel, Left/Right Live Vehicle Tuning Lab (`12` sliders + Drivetrain selector + `Defaults`), and Bottom Instrument Cluster (Speed, Gear/Mode, RPM bar, THR/BRK/Steer bars, 2D G-G circle plot, Pitch/Roll/Surface readouts).
