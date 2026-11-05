# GEMINI.md — Torque Lab Native (AI Context & Project Memory)

> **Purpose of this file:** This document is the single-source-of-truth project memory for **Gemini** and other AI coding agents. Read this file **first** when starting a new session or switching models—it provides the complete architecture, file map, coordinate/unit conventions, physics pipeline, hardware/driver quirks on this machine, and verified benchmark numbers so you do not need to scan the entire codebase.

---

## 1. Project Identity & Non-Negotiable Engineering Rules

- **Project Name:** **Torque Lab Native** (*Project Torque — 6-DOF Vehicle Dynamics Simulator*)
- **Tech Stack:** Native **C++20**, **OpenGL 3.3 Core Profile**, **GLSL 330 core**, **GLFW 3.4**, **GLAD**, **Bullet Physics (`btDiscreteDynamicsWorld`)**, **Dear ImGui**, and **CMake** (Visual Studio 2022 / MSVC Release build).
- **Migration Status:** 100% migrated from the original browser-based TypeScript/Three.js/Rapier3D application to native C++20/OpenGL. All legacy web/Node/TypeScript files have been removed from the workspace.
- **Core Physical Rule (Non-Negotiable):**
  - **Every vehicle behavior must emerge from coherent physical laws** (forces, torques, mass, rotational inertia, tire-ground slip, friction-circle saturation, suspension compression/rebound, and collision impulse transfer).
  - **NEVER** introduce fake animations, arbitrary speed multipliers, unphysical engine power multipliers, or unjustified gravity hacks.

---

## 2. Quick Commands (Build, Run, Test, Diagnostics)

All commands run from the workspace root: `d:\# My Folder\My Work\Visual Studio script\WebApplications\TorqueLab`

```powershell
# 1. Build both the main application and the automated test runner (Release)
cmake --build build --config Release --target TorqueLabNative TorqueLabNativeTests
# Or with GCC 14 (UCRT64 MinGW):
cmake --build build_gcc -j 4

# 2. Run the interactive desktop application (1440x900 OpenGL 3.3 window)
.\build\Release\TorqueLabNative.exe

# 3. Run the 41-suite automated unit, validation & controlled physics test harness
.\build_gcc\TorqueLabNativeTests.exe

# 4. Run full-pipeline acceleration telemetry table + 600-frame live OpenGL timing diagnostics
.\build\Release\TorqueLabNative.exe --diag

# 5. Run headless OpenGL + physics smoke test (CI / quick sanity check)
.\build\Release\TorqueLabNative.exe --smoke-test
```

---

## 3. Coordinate System, Units & Sign Conventions

All physics and rendering code operates strictly in **SI metric units (`m`, `kg`, `s`, `N`, `N·m`, `rad`, `rad/s`)** in a **Right-Handed World & Chassis Coordinate Frame**:

| Quantity / Axis | Convention | Sign / Notes |
| :--- | :--- | :--- |
| **Chassis Local $+X$** | **Left** (`basis.left`) | Front-Left (`FL`) & Rear-Left (`RL`) mounts have $x = +\text{trackWidth}/2 = +0.79\text{ m}$ |
| **Chassis Local $+Y$** | **Up** (`basis.up`) | Gravity acts along World $-Y$ ($g = 9.81\text{ m/s}^2$) |
| **Chassis Local $+Z$** | **Forward** (`basis.forward`) | Front axle is at $z > 0$, rear axle is at $z < 0$ |
| **Wheel Indexing `[0..3]`** | `0 = FL`, `1 = FR`, `2 = RL`, `3 = RR` | Fixed across `VehicleDynamics`, `Drivetrain`, `BrakeSystem`, `VehicleRendererGL`, and `ImGuiDashboard` |
| **Pitch Angle (`pitchRad`)** | Extracted via `getVehicleEulerAngles()` | **Positive ($>0$) = Rear Squat / Nose-Up** (under acceleration); **Negative ($<0$) = Front Dive / Nose-Down** (under braking) |
| **Roll Angle (`rollRad`)** | Extracted via `getVehicleEulerAngles()` | **Positive ($>0$) = Right Body Roll** (during a left turn with $+X$ lateral acceleration) |
| **Steering Angle** | `centerSteeringAngle`, `w.steeringAngle` | **Positive ($>0$) = Left Turn** (`A` / `Left Arrow`); **Negative ($<0$) = Right Turn** (`D` / `Right Arrow`) |

---

## 4. Complete Workspace File Map

### `src/physics/` — Math Primitives, Bullet World & Surface Registry
- [`src/physics/Math3D.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/Math3D.h): Double-precision `Vec3`, `Quat`, column-major float `Mat4` (for OpenGL), `VehicleBasis` (`forward`, `up`, `left`, `right`, `down`), `EulerAngles` (`yaw`, `pitch`, `roll`), and scalar helpers (`clamp`, `lerp`, `degToRad`, `radToDeg`).
- [`src/physics/CollisionMeshGeometry.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/CollisionMeshGeometry.h): CPU-retained raw mesh storage (`RawMeshGeometry`), convex decomposition pieces (`ConvexDecompositionPiece`), collision pipeline configuration (`CollisionPipelineSettings`), strategy enum (`CollisionAlgorithmStrategy`), and generated collision container (`GeneratedVehicleCollision`).
- [`src/physics/VehicleCollisionGenerator.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/VehicleCollisionGenerator.h) & [`VehicleCollisionGenerator.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/VehicleCollisionGenerator.cpp): Generic model-derived collision generation engine. Performs spatial voxel decimation, V-HACD volumetric decomposition, spatial topological clustering via `btConvexHullComputer`, automatic vehicle geometry derivation (wheelbase, track width, wheel radius, ground clearance), disk caching (`model/cache/<id>_collision.json`), and Bullet `btCompoundShape` synthesis.
- [`src/physics/SurfaceSystem.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/SurfaceSystem.h) & [`SurfaceSystem.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/SurfaceSystem.cpp): Defines the 6 road surfaces (`DryAsphalt` $\mu=1.00$, `WetAsphalt` $\mu=0.68$, `Gravel` $\mu=0.54$, `Ice` $\mu=0.18$, `Grass` $\mu=0.42$, `RumbleStrip` $\mu=0.88$) with friction, rolling-resistance, cornering-stiffness, and procedural roughness parameters. Maps Bullet collider `userIndex` handles to surface properties.
- [`src/physics/PhysicsWorld.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/PhysicsWorld.h) & [`PhysicsWorld.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/physics/PhysicsWorld.cpp): Owns `btDiscreteDynamicsWorld`, fixed `120 Hz` (`fixedDt = 1/120 s`, `maxSubSteps = 12`) accumulator loop (`updateFrame`, `stepFixed`), the Proving Ground track layout (`buildProvingGround`), and the 7 teleport spawn presets (`getProvingGroundPresets`). Supports switching vehicles with model-derived collision shapes.

### `src/vehicle/` — 6-DOF Vehicle Dynamics Subsystems
- [`src/vehicle/VehicleConfig.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleConfig.h) & [`VehicleConfig.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleConfig.cpp): `VehicleConfig` presets:
  - `createBmwM4Config()` (Factory M4 Coupe: S55/S58 3.0L twin-turbo I6, $550\text{ N}\cdot\text{m}$, 6-speed RWD, mass $1600\text{ kg}$, wheelbase $2.76\text{ m}$, track $1.58\text{ m}$, 220 kPa front / 240 kPa rear pneumatic pressures, 0.24m rim radius).
  - `createDodgeChallengerConfig()` (Authentic 1970 Dodge Challenger R/T: 426 cu in Street Hemi V8, $664\text{ N}\cdot\text{m}$ peak torque, 4-speed manual gearbox, Dana 60 Sure-Grip LSD 3.54:1, mass $1680\text{ kg}$, 54% front distribution, wheelbase $2.794\text{ m}$ [$110.0\text{ in}$], track $1.56\text{ m}$, vintage muscle bias-ply tires and softer rear ARB for authentic launch squat, 195 kPa / 210 kPa pressures, 0.19m rim radius).
  - `createProceduralCoupeConfig()` (`mass = 1500 kg`, `wheelbase = 2.70 m`, `trackWidth = 1.60 m`, `frontWeightDistribution = 0.52`, 220 kPa / 230 kPa pressures, 0.23m rim radius).
- [`src/vehicle/PneumaticTire.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/PneumaticTire.h) & [`PneumaticTire.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/PneumaticTire.cpp): Full structural pneumatic tyre simulation engine. Implements Kelvin-Voigt viscoelastic carcass compliance, pressure-dependent radial/lateral/longitudinal stiffness ($k_r(p) = k_{r,0} + C_p (p - p_0)$), multi-point contact patch discretization across circumferential and lateral grids with Hertzian-like parabolic weighting, series suspension/tyre compliance integration, lateral carcass cornering shear $\delta_y$, longitudinal shear $\delta_x$, Poisson sidewall bulge ($\Delta w = \nu \cdot \delta_r$), pneumatic trail self-aligning torque ($M_z = t_p \cdot F_y$), thermal energy balance (frictional dissipation + hysteresis heating vs convective/conductive cooling), tread wear accumulation, and puncture air leakage dynamics.
- [`src/vehicle/WheelState.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/WheelState.h): Per-wheel state (`WheelState`) storing raycast contact, suspension compression/velocity/forces, slip ratio $\kappa$, slip angle $\alpha$, longitudinal/lateral tire forces, angular velocity $\omega$, rotation angle, drive/brake torques, and the physical `PneumaticTireModel` instance.
- [`src/vehicle/VehicleBody.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleBody.h) & [`VehicleBody.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleBody.cpp): Wraps the chassis `btRigidBody` + `btCompoundShape` (lower body box + upper greenhouse box offset relative to the configurable Center of Mass). Supports live CoM and mass updates (`syncMassProperties`), world-point force application (`applyForceAtWorldPoint`), and exact analytical + Bullet raycasting (`castRayExactWorld`).
- [`src/vehicle/SuspensionSystem.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/SuspensionSystem.h) & [`SuspensionSystem.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/SuspensionSystem.cpp): Hooke spring ($F_s = -k \Delta x$), asymmetric bound/rebound damper ($c_{\text{bound}} \neq c_{\text{rebound}}$), progressive quadratic bump stop, and front/rear axle anti-roll bars (`computeAxleAntiRollForce`).
- [`src/vehicle/TireModel.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/TireModel.h) & [`TireModel.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/TireModel.cpp): Longitudinal slip ratio $\kappa$, lateral slip angle $\alpha$, load-sensitive friction coefficient $\mu(F_z)$, Pacejka Magic Formula $y(x) = D \sin(C \arctan(Bx - E(Bx - \arctan(Bx))))$, and combined-slip friction-circle saturation ($\sqrt{F_x^2 + F_y^2} \le \mu F_z$).
- [`src/vehicle/Drivetrain.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/Drivetrain.h) & [`Drivetrain.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/Drivetrain.cpp): Piecewise-linear engine torque curve (`sampleEngineTorqueCurve`), closed-throttle engine braking (`computeEngineBrakingTorque`), automatic/manual shift state machine, launch clutch slip engagement, soft fuel-cut rev limiter at `maxRpm`, RWD/FWD/AWD layout distribution, and speed-sensing Limited-Slip Differential (`distributeAxleTorqueWithLSD`).
- [`src/vehicle/SteeringSystem.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/SteeringSystem.h) & [`SteeringSystem.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/SteeringSystem.cpp): Speed-sensitive steering limit and Ackermann inner/outer front wheel geometry (`computeAckermannWheelAngles`).
- [`src/vehicle/BrakeSystem.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/BrakeSystem.h) & [`BrakeSystem.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/BrakeSystem.cpp): Front/rear biased service brake torques (`brakeBiasFront = 0.64`) and rear-only mechanical handbrake (`handbrakeTorque = 3400 N·m`).
- [`src/vehicle/Aerodynamics.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/Aerodynamics.h) & [`Aerodynamics.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/Aerodynamics.cpp): Quadratic aerodynamic drag ($F_d = \frac{1}{2}\rho C_d A v^2$), aerodynamic downforce ($F_L = \frac{1}{2}\rho C_L A v^2$), and per-wheel rolling resistance.
- [`src/vehicle/VehicleDynamics.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleDynamics.h) & [`VehicleDynamics.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleDynamics.cpp): Master 6-DOF vehicle orchestrator (`step`, `postWorldStep`, `getInterpolatedTransforms`, `getTelemetrySnapshot`, `resetVehicle`).

### `src/renderer/`, `shaders/`, `src/ui/`, `src/input/`, `src/app/`
- [`src/renderer/Shader.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/Shader.h) & [`Shader.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/Shader.cpp): GLSL shader compiler/linker with uniform location caching.
- [`src/renderer/Mesh.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/Mesh.h) & [`Mesh.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/Mesh.cpp): VAO/VBO/EBO procedural mesh generators (boxes, cylinders, rings, planes).
- [`src/renderer/ModelLoaderGLB.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ModelLoaderGLB.h) & [`ModelLoaderGLB.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ModelLoaderGLB.cpp): Single-header `cgltf`-based binary glTF 2.0 / GLB loader. Parses multi-node hierarchies into OpenGL VAOs, maps nodes to semantic vehicle roles (Chassis, 4 independent wheel assemblies, calipers, steering wheel), parses PBR metallic/roughness & transparent glass materials, reads optional JSON manifests (`model/BMW_M4_manifest.json`, `model/Dodge_Challenger_manifest.json`), supports embedded binary glTF textures, root scale & yaw rotation, and automatic multi-wheel quadrant partitioning (`ModelPartRole::Wheel_All`) for models whose wheels were authored as a single merged mesh.
- [`src/renderer/ShadowMapFBO.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ShadowMapFBO.h) & [`ShadowMapFBO.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ShadowMapFBO.cpp): `2048x2048` depth framebuffer for directional sun shadows.
- [`src/renderer/TrackRendererGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/TrackRendererGL.h) & [`TrackRendererGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/TrackRendererGL.cpp): Procedural grid/surface textures, Proving Ground meshes, 25m distance marker posts, and handling ring.
- [`src/renderer/VehicleRendererGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/VehicleRendererGL.h) & [`VehicleRendererGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/VehicleRendererGL.cpp): Master vehicle renderer with hot-swapping between multiple loaded 3D models (`BMW M4 Coupe`, `1970 Dodge Challenger R/T`, and the procedural sports coupe). Supports 2-pass rendering (opaque + transparent glass), reactive brake/reverse lights, suspension struts, and a 3D visual alignment debug overlay (`renderVisualDebug`).
- [`src/renderer/DebugOverlayGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/DebugOverlayGL.h) & [`DebugOverlayGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/DebugOverlayGL.cpp): 3D force vector gizmos (normal load $F_z$, longitudinal $F_x$, lateral $F_y$, velocity vector, and CoM marker) + rear-wheel dynamic skidmarks.
- [`src/renderer/CameraControllerGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/CameraControllerGL.h) & [`CameraControllerGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/CameraControllerGL.cpp): 4 camera modes (`Follow`, `Orbit`, `Suspension`, `Hood`).
- [`src/ui/ImGuiDashboard.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/ui/ImGuiDashboard.h) & [`ImGuiDashboard.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/ui/ImGuiDashboard.cpp): Top bar (zone presets `1..7`, interactive car selection dropdown combo, camera/gizmo/telemetry toggles, physics verifier button), left live tuning panel (vehicle selection dropdown + radio buttons, alignment debug checkbox, 12 real-time sliders + drivetrain layout selector), right telemetry panel (force balance summary, 4-wheel table, 4-band real-time oscilloscope), and bottom instrument cluster (digital speedometer, RPM bar, pedal bars, G-G diagram).
- [`src/input/ControllerConfig.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/ControllerConfig.h) & [`ControllerConfig.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/ControllerConfig.cpp): Logical vehicle action enum, Forza Horizon default mappings, Xbox/PlayStation/Nintendo button glyph adaptivity, and persistent JSON configuration (`settings/controller_config.json`).
- [`src/input/VibrationManager.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/VibrationManager.h) & [`VibrationManager.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/VibrationManager.cpp): Dynamic Windows XInput force-feedback rumble pipeline. Purely event-driven haptics inspired by Forza Horizon: collision impacts with non-linear impulse scaling & lateral directionality, suspension events (jump landings, bump-stop bottom-out, sharp curb strikes), underside chassis scrape buzzing, smooth cosine envelope decay, and collision deduplication. Zero vibration from engine RPM, throttle, or smooth cruising.
- [`src/input/InputControllerGLFW.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/InputControllerGLFW.h) & [`InputControllerGLFW.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/InputControllerGLFW.cpp): Progressive keyboard + baseline-calibrated multi-gamepad input handler with continuous analog steering, trigger normalization, right-stick look-around, interactive rebinding, hotplugging, and car cycling (`RB` / `B`).
- [`src/app/Application.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/app/Application.h), [`Application.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/app/Application.cpp), [`main.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/app/main.cpp): Lifecycle manager and CLI entry point (`--smoke-test`, `--diag`).
- [`tests/native_validation_tests.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/tests/native_validation_tests.cpp): 41 automated test suites (`Unit 1–28`, `Validation Tests A–J`, `Controlled Tests 1–3`).

---

## 5. Critical Machine / Hardware Context & Resolved Gotchas (DO NOT REGRESS)

1. **Virtual HID Gamepad on Host PC (`Monect Hid Device`):**
   - On this machine, `glfwJoystickIsGamepad(GLFW_JOYSTICK_1)` returns `1` for `'Monect Hid Device'`, whose idle axes all report `0.000` (`LX=0, LY=0, RX=0, RY=0, LT=0, RT=0`).
   - **Why it matters:** Standard XInput triggers rest at `-1.0f`. Naive `(axis + 1.0) * 0.5` mapping turns `LT = 0.000` into a permanent **50% brake input (`brake = 0.50`)**, locking the vehicle at `0.03 m/s`!
   - **How it is solved in `InputControllerGLFW`:** We capture the initial resting baseline of all 6 gamepad axes on connection (`m_gamepadBaselineAxes`) and only mark an axis active (`m_gamepadAxisActivated[i] = true`) after it deviates by `> 0.12f` from its resting value. **Never remove this baseline activation guard.**

2. **Low-Speed Parking Hold Guard in `VehicleDynamics::step()`:**
   - The stationary parking-hold damper (`if (std::hypot(vx, vy) < 0.8 && w.brakeTorque > 50.0 && effectiveThrottle < 0.05)`) **must** keep the `effectiveThrottle < 0.05` condition so braking hold never opposes active throttle launch.

3. **Coupled Driven-Wheel Reflected Engine Inertia & Soft Redline Limit:**
   - In `VehicleDynamics::step()`, driven wheels include reflected engine inertia (`0.25 * clutchEngagement * engineInertia * totalRatio^2 / numDrivenWheels`) when accelerating and smoothly taper positive `driveTorque` as wheel angular speed approaches $\omega_{\text{redline}} = \frac{\text{maxRpm} \cdot \pi / 30}{|G_{\text{total}}|}$.
   - In `Drivetrain::updateDrivetrain()`, when `state.revLimiterActive` is true and `throttle > 0.05`, `state.engineTorqueNm` cuts cleanly to `0.0` instead of slamming full closed-throttle vacuum braking (`-95.8 N·m`).

4. **Integrated GPU (`AMD Radeon(TM) Vega 3 Graphics`) OpenGL Driver Behavior:**
   - Keep `glfwWindowHint(GLFW_SAMPLES, 0)` and `glfwSwapInterval(0)` in `Application::init()`, and always unbind `GL_TEXTURE1` before `m_shadowFBO.beginPass()` in `Application::renderFrame()`.
   - Enabling default-framebuffer MSAA or leaving the shadow depth texture bound to `GL_TEXTURE1` while rendering into `m_shadowFBO` triggers a `45–48 ms` driver stall inside `SwapBuffers` (`~19 FPS`). With the current configuration, the application runs at **`210–220 FPS` (`~4.5 ms` frame time)**.

5. **Chase Camera (`CameraMode::Follow`) Heading Damping:**
   - `CameraControllerGL::update()` damps the horizontal forward direction vector (`m_smoothedForward`) and vertical height (`m_smoothedHeight`) around the vehicle's interpolated position `pos` rather than lerping `m_position` in world space. This prevents steady-state camera distance lag at high speed (`CamDist` remains `6.61 m` at all speeds).

6. **Model-Derived Collision Geometry & Full 10-Hull Chassis/Wheel Alignment:**
   - Raw glTF mesh vertices extracted from Blender models have their origin at ground level $(0,0,0)$ and may include complex scene graph parent node hierarchies (armatures, collections, and scale factors).
   - In `ModelLoaderGLB.cpp`, all nodes are resolved via `cgltf_node_transform_world(node, worldMatrix)`, ensuring subparts with ancestor scales (such as bumper honeycombs and windshield wipers) are placed in exact sub-centimeter positions rather than floating off in space.
   - Wheel assemblies are extracted into dedicated tight wheel collision hulls (`WheelFL_Hull`, `WheelFR_Hull`, `WheelRL_Hull`, `WheelRR_Hull`), each filtering outlier hub links to form true cylindrical $0.30\text{ m}$ radius, $0.24\text{ m}$ width wheel colliders.
   - 100% of the imported car geometry is converted into 10 watertight convex collision hulls: 6 chassis body pieces (`FrontBumper_Hull`, `HoodFenders_Hull`, `Greenhouse_Hull`, `ChassisTub_Hull`, `RearDeck_Hull`, `RearBumper_Hull`) plus 4 dedicated wheel/tire hulls.
   - In `VehicleCollisionGenerator::generateFromModel()`, the raw chassis meshes are transformed into **Chassis Physics Space** via `Mat4::fromTranslation(model.chassisCenterOffset)` before volumetric convex decomposition.
   - This ensures child hulls in Bullet's `btCompoundShape` align 1:1 with the visual car body at $(T_{\text{chassis}} \times P_{\text{piece}})$, eliminating floating collision cages and bumper phasing.
   - Continuous Collision Detection (`setCcdMotionThreshold(0.08f)` and `setCcdSweptSphereRadius(0.20f)`) is active on `m_rigidBody`, preventing high-speed obstacle tunneling.
   - **Collision Bouncing Elimination:** Bullet solver is tuned with `m_splitImpulse = 1` (position-only contact push-out without velocity addition), `m_globalCfm = 0.0f` (eliminates spongy penetration), and chassis restitution $0.02f$. Suspension raycasts in `VehicleDynamics` skip non-driveable obstacle walls (`surfaceSystem.isColliderDriveable`), and bump-stop penetration is clamped to avoid upward load spikes on impact.

---

7. **Physics Event-Driven Controller Haptics (RPM Vibration Elimination):**
   - Controller vibration is strictly event-driven. Engine RPM, engine pitch/audio, throttle application, wheelspin slip, and smooth cruising produce **strictly 0.0% vibration**.
   - Haptics are triggered exclusively by:
     - **Barrier/Obstacle Collisions:** Derived from rigid-body velocity jump $|\Delta \vec{v}|$, contact normal impulse $J$, and contact lifetime. Resting contact against a wall settles to 0.0% rumble after initial impact. Collision deduplication suppresses contact chatter.
     - **Suspension Events:** Hard jump landings ($v_{\text{down}} > 0.6\text{ m/s}$, heavy motor), progressive bump-stop bottoming out ($w.\text{bumpStopForce} > 800\text{ N}$, heavy motor), sharp curb/bump strikes ($dL/dt < -2.2\text{ m/s}$, light motor), and underside chassis scrapes (light motor buzz).
     - **Lateral Directionality:** Left-side impacts bias toward the left vibration motor; right-side impacts bias toward the right motor.
     - **Decay & Controls:** Cosine envelope decay with master intensity, collision sensitivity, and suspension sensitivity sliders in ImGui and `settings/controller_config.json`.

8. **Atmospheric Environment Overhaul & Graphics Quality Presets:**
   - **Atmospheric Sky Dome & Color Calibration (`SkyRendererGL`):** Renders a procedural daytime sky dome at far clip depth ($z = w$, `GL_LEQUAL`, `glDepthMask(GL_FALSE)`). Calibrated Rayleigh scattering constants produce rich sapphire zenith (`rgb(0.04, 0.16, 0.65)`), clear azure mid-sky (`rgb(0.12, 0.38, 0.85)`), and luminous cerulean horizon blue (`rgb(0.38, 0.62, 0.94)`) with zero dull gray haze. Mie scattering models a sharp solar disc and multi-octave aureole corona.
   - **Double Gamma Elimination:** The Narkowicz ACES filmic tone reproduction curve (`acesToneMap`) natively maps linear HDR inputs to non-linear sRGB display output. Applying an extra `pow(x, 1/2.2)` on top resulted in double gamma compression (lifting dark shadows to 0.13 gray, flattening contrast, and washing out color saturation). Removing the redundant second gamma pass across both `phong_shadow.frag` and `sky.frag` restored deep, vibrant colors and authentic contrast.
   - **glTF Decal Alpha Compositing:** Vehicle liveries with transparent backgrounds (e.g. BMW M4 M-stripes and decals) blend over underlying body paint via `mix(uBaseColor, texCol.rgb, texCol.a)` rather than multiplicative blending (`albedo *= texCol.rgb; alpha *= texCol.a;`), preventing transparent decal boundaries from turning body paint black or invisible.
   - **Foreground Fog Exclusion (0–400m):** Distance fog starts strictly at $400\text{ m}$ (`fogDist = max(dist - 400.0, 0.0)`), leaving the vehicle, straightaway, skidpad, and track markers 100% fog-free and razor sharp. Beyond $400\text{ m}$, distant terrain skirts smoothly and seamlessly dissolve into the sky dome horizon using tonemapped horizon color (`acesToneMap(uFogColor * exposure)`).
   - **PBR Direct Light Energy Balance:** Direct diffuse lighting preserves unscaled sunlight irradiance ($L_o = (1 - k_S) \cdot \text{diffuse} \cdot \vec{L}_{\text{sun}} \cdot (\vec{N} \cdot \vec{L})$), avoiding rogue $1/\pi$ light reduction when sun color is supplied as illuminance, and automotive metallic paint basecoat diffuse is preserved under clearcoat (`diffuseColor = albedo * (1.0 - metalness * 0.75)`).
   - **Graphics Quality Presets (`GraphicsQuality`):**
     - **`Maximum`:** Full atmospheric Rayleigh/Mie sky dome, $2048 \times 2048$ shadow map with 8-tap Poisson disk PCF filtering, full HDR IBL reflections + diffuse irradiance, anti-banding dither, and expansive horizon skirt.
     - **`High`:** Baseline environment, $2048 \times 2048$ shadow map with 4-tap Poisson disk PCF, standard IBL, atmospheric sky dome.
     - **`Medium`:** Performance-balanced, $1024 \times 1024$ shadow map with fast 2-tap PCF, 3-stop sky gradient, basic IBL.
     - **`Low`:** Fast fallback, dynamic shadows disabled, fast 2-stop sky gradient, hemispherical ambient lighting without expensive IBL texture lookups.
   - **Runtime Hot-Swapping:** Live switching via Dear ImGui top-bar combo box with zero memory leaks, shader recompile hitches, or GPU stalls.

---

## 6. Verified Physical Benchmarks (`28 / 28` Suites Passing)

Any future code change must keep all 28 suites in `.\build\Release\TorqueLabNativeTests.exe` green:

- **Static Equilibrium (`Test A`):** $\sum F_z = 14715.0\text{ N}$ ($0.000\%$ error vs $mg = 1500 \times 9.81 = 14715.0\text{ N}$), ride height $= 0.4910\text{ m}$, suspension compression $= 0.1093\text{ m}$.
- **Full-Throttle Launch (`Test B` & `--diag`):**
  - At $t = 2.0\text{ s}$: $v = 10.60\text{ m/s}$ ($38.2\text{ km/h}$), $a_x = +5.34\text{ m/s}^2$, rear squat pitch $= +0.798^\circ$, rear axle load transfer $\Delta F_{z,\text{rear}} = +1312\text{ N}$.
  - $0\text{--}100\text{ km/h}$ ($27.78\text{ m/s}$) achieved in **$5.84\text{ s}$**; reaches **$143.6\text{ km/h}$ ($39.88\text{ m/s}$)** at $t = 10.0\text{ s}$ ($Z = 229.29\text{ m}$).
- **Service Braking (`Test C`):** From $20\text{ m/s}$, $v(0.5\text{ s}) = 16.94\text{ m/s}$, deceleration $a_x = -9.82\text{ m/s}^2$, nose-down dive pitch $= -1.609^\circ$, $F_{z,\text{front}} = 9970\text{ N}$ vs $F_{z,\text{rear}} = 4704\text{ N}$.
- **Cornering Roll (`Test D`):** Left turn roll $= +2.30^\circ$ ($a_y = +9.70\text{ m/s}^2$); Right turn roll $= -2.27^\circ$ ($a_y = -9.55\text{ m/s}^2$).
- **Surface Hierarchy (`Test E`):** Dry asphalt launch $v = 6.01\text{ m/s}$, stopping distance $= 11.36\text{ m}$ vs Ice launch $v = 1.24\text{ m/s}$, stopping distance $= 51.34\text{ m}$.
- **Timestep Independence (`Test I`):** $30\text{ FPS}$, $60\text{ FPS}$, and $120\text{ FPS}$ produce identical trajectories ($Z = 9.126\text{ m}, V = 9.790\text{ m/s}$ after $2.0\text{ s}$).
- **Controlled Force & Scale Sanity (`Controlled 1–3`):**
  - $F = 5800\text{ N}$ on $m = 1500\text{ kg}$ $\implies a = 3.868\text{ m/s}^2$ (expected $3.867\text{ m/s}^2$).
  - Constant $v = 25.00\text{ m/s}$ for $t = 2.000\text{ s}$ $\implies \Delta Z = 50.002\text{ m}$ (expected $50.000\text{ m}$).

---

## 8. Automated Compile-Time & Runtime Collision Baking Pipeline

1. **Automatic Build-Time Collision Generation (`CMakeLists.txt` Post-Build Custom Command):**
   - Whenever compiling (`cmake --build build --config Release`), CMake runs a post-build step: `TorqueLabNative --bake-collisions`.
   - The baker operates **100% headlessly on pure CPU** (`loadGpuResources = false` in `ModelLoaderGLB::loadFromFile`), requiring zero OpenGL or window initialization.
   - It scans `model/` for all `.glb` and `.gltf` files, checking their timestamps against `model/cache/<vehicle_id>_collision.json`.
   - If a model is new or was recently modified/exported from Blender (`last_write_time(glb) > last_write_time(cache)` or companion manifest was updated), it automatically executes volumetric convex decomposition (`btConvexHullComputer` / V-HACD), derives vehicle metrics (wheelbase, track width, wheel radius, ground clearance), writes the updated JSON cache, and syncs it to the build output directory.
   - If up-to-date, it skips re-computation in $<1\text{ ms}$.

2. **Runtime Stale Cache Invalidation:**
   - At runtime, `VehicleCollisionGenerator::generateFromModel()` executes the same timestamp validation. If a user modifies or drops in a `.glb` while running or between runs without recompiling, the game detects the stale cache and regenerates it on the fly.

3. **Dynamic Multi-Vehicle Auto-Discovery & Selection:**
   - `VehicleRendererGL::init()` dynamically scans `model/` for all `.glb` files and registers them into `m_availableVehicles` alongside `"procedural_coupe"`.
   - The Dear ImGui top-bar dropdown combo, left tuning panel combo, and radio buttons dynamically render all discovered vehicles.
   - Gamepad `RB` and Keyboard `B` dynamically cycle through all available vehicles.
   - Any newly added car model immediately receives physical collision hulls, derived wheel centers, wheelbase, and track width without requiring hardcoded vehicle configurations. Custom powertrain/mass parameters can be live-tuned and saved to `model/<id>_vehicle.json`.

1. **Decoupled Vehicle Configuration System (`VehicleConfig.h` / `VehicleConfig.cpp`):**
   - Pure physical parameters (`mass`, `wheelbase`, `trackWidth`, `inertiaTensor`, spring/damper rates, drivetrain, brakes) are decoupled from rendering models.
   - Per-vehicle JSON specifications stored in `model/bmw_m4_vehicle.json` and `model/procedural_coupe_vehicle.json`. Supports runtime serialization (`saveVehicleConfigToJson`) and deserialization (`loadVehicleConfigFromJson`).
   - Seamless live vehicle switching via `PhysicsWorld::switchVehicle(newConfig)` with zero memory leaks, full Bullet rigid body reconstruction, and suspension ray synchronization.

2. **Accurate Visual Appearance, Materials & Decal Composition:**
   - The Blender model is an **Alpine White BMW M4 with authentic BMW M tri-color racing stripes**, carbon fiber trim, chrome highlights, transparent tinted glass, and ruby red brake lights.
   - 4 packed textures extracted from `.blend` file into `model/textures/`:
     - `Front_Bumper_Text.png` (2048x2048 RGBA decal on front bumper)
     - `Front_Hodd_Text.png` (2048x2048 RGBA decal on hood)
     - `Stripes_Wings.png` (2048x2048 RGBA M-stripes on front fenders / doors)
     - `600px-BMW.svg.png` (600x600 RGBA BMW roundel emblem on wheels and hood)
   - Fragment shader (`phong_shadow.frag`) composites decal textures over Alpine White base paint (`#DCE0E6`) via `albedo = mix(albedo, texColor.rgb, texColor.a)`.

3. **Ground Clearance & Axis Alignment:**
   - Model origin $(0,0,0)$ in `BMW_M4.glb` sits at ground level under the tire contact patches.
   - Offset `visualOffset = Vec3{0.0, -0.477, -0.1475}` aligns the model's chassis floor with exact real-world ground clearance ($12.7\text{ cm}$), sitting flush on the road with wheel hubs centered in the wheel arches.

4. **Tapered Compound Collision Hull (`CollisionShapeType::TaperedCompound`):**
   - Replaced oversized rectangular collision box with an aerodynamic tapered compound shape in `VehicleBody.cpp`:
     - **Central Passenger Tub:** $Z \in [-0.40 \cdot hz, +0.40 \cdot hz]$, width $2 \cdot hx$ ($1.84\text{ m}$)
     - **Tapered Front Nose & Curved Bumper (`btConvexHullShape`):** Tapers from $1.75\text{ m}$ down to $1.44\text{ m}$ at bumper corners with chamfered apex tip, eliminating the phantom 25–30 cm corner collision gap!
     - **Tapered Rear Bumper (`btConvexHullShape`):** Tapers to $1.55\text{ m}$ at rear bumper tip.
     - **Upper Cabin Greenhouse:** Half-extents matching glass cabin envelope.
   - Real-time 3D wireframe visualization toggle in `DebugOverlayGL` (`showCollisionWireframe`) renders the exact Bullet collision boundary cage in emerald green.

5. **Generic Model-Derived Shape-Accurate Vehicle Collision Pipeline (`VehicleCollisionGenerator`):**
   - Replaces hardcoded box configurations with a generic automatic collision generation pipeline derived directly from loaded 3D vehicle models (GLTF/GLB or procedural meshes).
   - Dynamic moving rigid bodies in Bullet strictly require convex shapes or compound shapes of convex hulls (`btCompoundShape` with `btConvexHullShape` children). Concave triangle meshes (`btBvhTriangleMeshShape`) are static-only in Bullet and fail on dynamic bodies.
   - The generator performs spatial voxel decimation from high-density visual vertices (e.g. 743k triangles) down to uniform surface vertices in $<5\text{ ms}$, then executes volumetric approximate convex decomposition (V-HACD) or spatial topological clustering via `btConvexHullComputer`.
   - Automatically decomposes the car into 10 shape-accurate watertight convex hulls: 6 chassis body pieces (`FrontBumper_Hull` following the curved nose contour, `HoodFenders_Hull`, `Greenhouse_Hull` following the authentic A-pillars and roofline, `ChassisTub_Hull`, `RearDeck_Hull`, `RearBumper_Hull`) plus 4 dedicated tire/wheel hulls (`WheelFL_Hull`, `WheelFR_Hull`, `WheelRL_Hull`, `WheelRR_Hull`). 100% of the imported model geometry is captured into physical collision.
   - Separates wheel geometry to automatically derive wheel centers, wheel radius ($0.30\text{ m}$), wheel width ($0.24\text{ m}$), wheelbase ($2.76\text{ m}$), track width ($1.58\text{ m}$), and ground clearance ($10.0\text{ cm}$) directly from 3D mesh coordinates. Outlier vertices in wheel asset origins are filtered from the chassis collision mesh, eliminating pointed front/rear bumper protrusion artifacts ($Z$ strictly spans $[-2.265, +2.330]\text{ m}$, total length $4.595\text{ m}$).
   - Automatically serializes and caches generated convex hulls to `model/cache/<vehicle_id>_collision.json` (cache version 5) for instantaneous subsequent loads ($<1\text{ ms}$).
   - 3D Wireframe Overlay (`DebugOverlayGL`) renders each convex child hull with distinct vibrant colors (emerald, cyan, amber, purple, pink, blue, lime, red) for clear visual inspection.
   - Real-time diagnostic telemetry and manual re-generation buttons are integrated into `ImGuiDashboard`.


---

## 9. Complete glTF 2.0 PBR Material, Texture Pipeline & IBL Lighting Engine

1. **Full glTF 2.0 Metallic-Roughness Asset Importer (`ModelLoaderGLB.cpp`):**
   - **Preservation of Authentic PBR Properties:** Reads `base_color_factor`, `metallic_factor`, `roughness_factor`, `emissive_factor`, and alpha modes (`Opaque`, `Mask`, `Blend`) directly from source glTF meshes. Eliminated destructive hardcoded overrides that previously forced all materials to Alpine White.
   - **Complete Texture Channel Resolution:** Resolves and caches embedded binary glTF textures (`bufferView`) and external textures across all standard channels:
     - Unit 0: **Base Color / Albedo** (`GL_SRGB8_ALPHA8`, sRGB color space)
     - Unit 1: **Directional Shadow Map** (`GL_DEPTH_COMPONENT24`, depth)
     - Unit 2: **HDR Environment Panoramic Map** (`GL_RGBA8` / sRGB, IBL)
     - Unit 3: **Tangent-Space Normal Map** (`GL_RGBA8`, Linear space) with `normalScale`
     - Unit 4: **Metallic-Roughness Map** (`GL_RGBA8`, Linear space; Green = Roughness, Blue = Metalness)
     - Unit 5: **Ambient Occlusion Map** (`GL_RGBA8`, Linear space; Red = AO) with `occlusionStrength`
     - Unit 6: **Emissive Map** (`GL_SRGB8_ALPHA8`, sRGB color space)
   - **Orthonormal Tangent Generation (`Mesh::computeTangents`):** Automatically computes smooth orthogonal tangents ($\vec{T}$) and bitangent handedness signs ($tw \in \{-1.0, 1.0\}$) across indexed triangle meshes using Gram-Schmidt orthogonalization when assets lack pre-authored tangents.

2. **Cook-Torrance Microfacet GGX PBR Shader Pipeline (`phong_shadow.vert` / `phong_shadow.frag`):**
   - Replaced legacy empirical Blinn-Phong lighting with industry-standard metallic-roughness BRDF:
     - **Normal Distribution Function (NDF):** Trowbridge-Reitz GGX ($D = \frac{\alpha^2}{\pi ((N \cdot H)^2 (\alpha^2 - 1) + 1)^2}$ where $\alpha = \text{roughness}^2$).
     - **Geometric Shadowing:** Smith-Schlick GGX ($G = G_1(N, V) \cdot G_1(N, L)$).
     - **Fresnel Reflectance:** Fresnel-Schlick ($F_0 = \text{mix}(0.04, \text{albedo}, \text{metalness})$) with roughness attenuation.
     - **Energy Conservation:** $k_D = (1 - F) \cdot (1 - \text{metalness})$ balances diffuse and specular lobes.
     - **Tangent-Space Normal Mapping:** Evaluates $3 \times 3$ TBN matrix with two-sided normal inversion (`gl_FrontFacing`).

3. **High-Dynamic-Range Environment Map & IBL (`EnvironmentMapGL`):**
   - Procedural automotive HDR environment panoramic texture ($512 \times 256$) featuring atmospheric sky gradient, sun corona, and 3 overhead studio softbox light banks that highlight vehicle body curvature and shoulder creases.
   - Trilinear mipmap filtering (`glGenerateMipmap(GL_TEXTURE_2D)`) maps surface `roughness` directly to reflection blur ($LOD = \text{roughness} \times \text{maxLod}$).
   - Diffuse irradiance ambient integration combined with ACES filmic tone reproduction and gamma 2.2 linear workflow.

4. **Real-Time PBR Material Inspector & Diagnostics Panel (`ImGuiDashboard`):**
   - 9 real-time shading diagnostic modes: Full PBR Lit, Albedo, Metalness, Roughness, Shading Normals, Ambient Occlusion, Emissive, Tangents, and Environment Specular.
   - Real-time Exposure ($0.2\text{--}3.0$) and IBL Multiplier ($0.0\text{--}3.0$) sliders.
   - Interactive breakdown table displaying all distinct materials in the active model, complete with live color swatches, roughness/metalness parameters, and texture map badges (`[Alb]`, `[Nrm]`, `[Met-Rough]`, `[AO]`, `[Ems]`, `[Blend]`, `[Mask]`, `[2-Sided]`).


---

## 10. Volumetric Tyre Collision, Kinematic Wheel Cylinders & Dynamic Geometry Derivation

1. **Volumetric Circumferential & Lateral Tire Collision Pipeline (`VehicleDynamics.cpp`):**
   - Replaced legacy 1D single-point raycasts with multi-point volumetric envelope contact queries: 27 circumferential and lateral probe vectors spanning outer tyre radius $R$ and tread width $W$ across inner shoulder (-40%), center (0%), and outer shoulder (+40%) slices.
   - Forward approach arc ($\theta \in [0^\circ..40^\circ]$) detects curbs, raised platforms, stones, and steps before the axle center reaches the obstacle ($Z_{\text{contact}} > Z_{\text{axle}}$).
   - Rear departure arc ($\theta \in [-25^\circ..0^\circ]$) provides continuous tire support when descending from raised ledges and ramps without abrupt drops.
   - Calculates effective circumferential contact normal $\vec{n} = \cos(\theta)\vec{u}_{\text{up}} - \sin(\theta)\vec{u}_{\text{fwd}}$, applying horizontal obstacle resistance force ($F_{\text{obs}}$) opposing forward travel into the obstacle, and drive-wheel climbing traction ($F_{\text{climb}}$) pulling the wheel up and over steps under throttle.

2. **Dedicated Physical Wheel Collision Objects (`btCylinderShapeX`):**
   - Removed static/frozen wheel hulls from the chassis `btCompoundShape`, eliminating rigid collision interference and obstacle snagging.
   - Created 4 dedicated Bullet collision objects using `btCylinderShapeX(wWidth * 0.5, wRadius, wRadius)`. In Torque Lab local frame ($+X$ Left, $+Y$ Up, $+Z$ Forward), the cylinder central axis along $+X$ aligns with the wheel rotation axle with zero rotational offset.
   - Flagged with `btCollisionObject::CF_NO_CONTACT_RESPONSE` and broadphase mask `btBroadphaseProxy::StaticFilter` so Bullet's dynamic solver never applies unwanted contact impulses or constraints to the chassis rigid body.
   - Exact world transforms ($T_{\text{wheel}} = \text{pos} + \text{rot}$) synchronized at 120 Hz with suspension stroke and Ackermann steering rotation.
   - `castRayExactWorld` and `ExcludeVehicleRayCallback` explicitly ignore all 4 wheel collision objects to ensure raycasts find the terrain and obstacle geometry with zero self-occlusion.

3. **Generic Model-Derived Wheel Geometry & Per-Axle Fitment (`ModelLoaderGLB.cpp`, `VehicleCollisionGenerator.cpp`, `VehicleConfig.cpp`):**
   - Wheel radius ($R$), width ($W$), pivot center, and track width are derived directly from actual 3D mesh vertex bounding boxes across model parts tagged with `ModelPartRole::Wheel_*`.
   - Populates per-axle dimensions: `frontTrackWidth`, `rearTrackWidth`, `frontWheelRadius`, `rearWheelRadius`, `frontWheelWidth`, and `rearWheelWidth`, supporting staggered front/rear fitments.
   - Automatically serialized and deserialized to/from JSON in vehicle configurations.

4. **3D Collision Wireframe & Telemetry Visualization (`DebugOverlayGL.cpp`, `ImGuiDashboard.cpp`):**
   - Wireframe cylinder rendering (`addCylinderXWireframe`) dynamically renders the exact physical `btCylinderShapeX` for each wheel at its true simulated position, compression, and steering angle in distinct Cyan (front) and Amber (rear) colors.
   - Visualizes circumferential obstacle contact points, resistance force arrows, and climbing traction vectors.
   - Interactive 4-wheel telemetry table highlights obstacle contact with live hit angle and force metrics.

---

## 11. Deformable Pneumatic Tyre Simulation, Multi-Point Contact Patch & Vertex Shader Deformation

1. **Pneumatic Tyre Carcass Mechanics & Series Compliance (`PneumaticTire.cpp`, `VehicleDynamics.cpp`):**
   - Structural Kelvin-Voigt viscoelastic carcass model with dynamic pressure dependence ($p \in [80, 400]\text{ kPa}$):
     $$k_r(p) = k_{r,0} + C_{p,r}(p - p_0), \quad k_y(p) = k_{y,0} + C_{p,y}(p - p_0), \quad k_x(p) = k_{x,0} + C_{p,x}(p - p_0)$$
   - Series spring equilibrium between suspension spring ($k_{\text{susp}}$) and tyre radial carcass ($k_{\text{tire}}$):
     $$\frac{1}{k_{\text{eff}}} = \frac{1}{k_{\text{susp}}} + \frac{1}{k_{\text{tire}}}, \quad \delta_r = \Delta x_{\text{total}} \cdot \frac{k_{\text{susp}}}{k_{\text{susp}} + k_{\text{tire}}}$$
   - Verified static equilibrium on flat ground with $0.000\%$ error ($F_z = 14715.0\text{ N}$).

2. **Distributed Multi-Point Contact Patch Discretization:**
   - Discretizes the rectangular tyre contact patch ($2 a \times W$) into runtime grids matching the collision fidelity modes:
     - **Performance Mode:** 3 samples ($3 \times 1$ along centerline).
     - **Balanced Mode:** 9 samples ($3 \times 3$ across inner shoulder, center, outer shoulder).
     - **Detailed Mode:** 15 samples ($5 \times 3$ grid).
   - Hertzian-like parabolic pressure profile: $w(u) = 0.20 + 0.80(1 - u^2)$, strictly normalized so $\sum w_j = 1.0$, guaranteeing total contact load identically equals axle normal load ($\sum F_{z, j} = F_z$).
   - Longitudinal contact moment arm incorporates pneumatic trail ($t_p = 0.22 \cdot a$), generating physical self-aligning torque ($M_z = t_p \cdot F_y$) applied directly to the chassis body.

3. **Strict Rigid Rim Protection in GLSL Shaders (`phong_shadow.vert`, `shadow_depth.vert`):**
   - Evaluates vertex radial distance in wheel local cross-section: $r = \sqrt{P_{\text{wheelLocal}, y}^2 + P_{\text{wheelLocal}, z}^2}$.
   - Rim threshold guard: If $r \le R_{\text{rim}}$, radial factor $f(r) = 0.0$ identically. Only outer rubber ($r \in [R_{\text{rim}}, R_0]$) undergoes deformation:
     $$P_{\text{local}}' = P_{\text{local}} + \text{radialSquash} \cdot \vec{u}_r + \text{sidewallBulge} \cdot \vec{u}_{\text{axle}} + \text{lateralShear} \cdot \vec{u}_y + \text{longitudinalShear} \cdot \vec{u}_z$$
   - Wheel hub, spokes, calipers, and rim beads remain completely rigid and pristine at all pressures and loads.

4. **Thermodynamics, Wear Accumulation & Puncture Leakage:**
   - First-law thermal energy balance: frictional sliding dissipation ($Q_{\text{slip}} = |\vec{F}_{\text{friction}} \cdot \vec{v}_{\text{slip}}|$) + cyclic carcass hysteresis ($Q_{\text{hyst}} = C_h \cdot \omega \cdot \delta_r \cdot F_z$) vs forced convective air cooling ($h_{\text{conv}} \propto 1 + 0.04 v$) and conductive ground transfer.
   - Archard-inspired abrasive wear accumulation: $\Delta \text{wear} = K_w \frac{F_{\text{friction}} \cdot v_{\text{slip}}}{H} \Delta t$.
   - Puncture orifice air leakage: $\dot{p} = -C_{\text{leak}} \sqrt{p - p_{\text{amb}}}$, causing observable carcass deflation, sidewall squashing, and increased rolling resistance.

5. **Dear ImGui Pneumatic Tyre Dashboard & 3D Debug Overlays:**
   - Real-time front and rear tyre pressure sliders ($100\text{--}350\text{ kPa}$ / $15\text{--}50\text{ PSI}$) with factory presets (`Factory M4`, `Dodge Track`, `High Speed`, `Comfort`).
   - Puncture / Repair simulation buttons with live deflation rates.
   - 4-wheel telemetry table showing deflection $\delta_r$, sidewall bulge $\Delta w$, lateral shear $\delta_y$, temperatures, wear, contact length, and self-aligning moments.
   - 3D overlay rendering flattened contact patch rectangular footprint box, longitudinal centerline, bulging sidewall profiles, and multi-point sample contact force vectors.
   - Live ground constraint status indicators: physical penetration $0.0\text{ mm}$, visual tread penetration $0.0\text{ mm}$, rigid rim protected.

6. **Ground Penetration Elimination & World-Space Deformation (`phong_shadow.vert`, `shadow_depth.vert`, `Mesh.cpp`):**
   - **Root Cause Diagnosis (Case C):** Series compliance lowers the physical wheel hub by $\delta_r$ ($Y_{\text{hub}} = R_0 - \delta_r$) so normal forces balance vehicle mass at $Y = 0.0$ ($0.000\%$ error). However, undeformed wheel radius $R_0$ placed bottom vertices at $Y = -\delta_r$ (e.g., $-2.2\text{ cm}$). When vertex deformation was computed in local `aPos` space using an empirical angular decay curve, the displacement dropped faster than the circular arc depth, causing tread vertices in the contact zone to sag up to $7\text{ mm}$ below the asphalt and appear clipped or phasing through the ground. Furthermore, imported glTF meshes had node hierarchies and scale offsets that distorted local radius evaluations.
   - **World-Space Invariant Constraint:** Vertices are transformed to world coordinates first ($P_{\text{world}} = uModel \cdot aPos$). The exact signed perpendicular distance to the road plane is calculated:
     $$d_{\text{ground}} = (P_{\text{world}} - P_{\text{ground}}) \cdot \hat{n}_{\text{ground}}$$
     When penetrating ($d_{\text{ground}} < 0$), vertices are lifted along $\hat{n}_{\text{ground}}$ by exactly $(-d_{\text{ground}} + \text{edgeRounding}) \cdot \eta_{\text{smooth}}$, guaranteeing that deformed contact tread vertices meet the road surface flush at $d_{\text{ground}} = 0.000\text{ m}$ without knife-edges or center craters.
   - **Dedicated Procedural Pneumatic Tyre Mesh (`Mesh::createPneumaticTireMesh`):** Built with 64 radial segments, 6 concentric sidewall rings, crowned tread band, and distinct bead seats, providing smooth vertex density across the deformation zone.
   - **Dielectric Carbon Black PBR Material:** `roughness = 0.82`, `metalness = 0.00`, `baseColor = #1c1f24`, applied across procedural wheels and manifests (`BMW_M4_manifest.json`, `Dodge_Challenger_manifest.json`).
   - **Directional Shadow Depth Parity (`shadow_depth.vert`):** World-space ground flattening and sidewall bulging are mirrored identically in the shadow pass, casting accurate deformed tyre silhouettes onto the track.
   - **Automated Verification (`Unit 28`):** Rigorously verifies zero penetration ($d_{\text{ground}} \ge -0.0001\text{ m}$), flush contact patch ($Y = 0.000\text{ m}$), strict rigid rim protection ($r \le R_{\text{rim}} \implies |\Delta P| = 0$), and monotonic curvature continuity.
