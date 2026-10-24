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

# 2. Run the interactive desktop application (1440x900 OpenGL 3.3 window)
.\build\Release\TorqueLabNative.exe

# 3. Run the 24-suite automated unit, validation & controlled physics test harness
.\build\Release\TorqueLabNativeTests.exe

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
- [`src/vehicle/VehicleConfig.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/VehicleConfig.h): `VehicleConfig` struct and `createDefaultVehicleConfig()` (`mass = 1500 kg`, `wheelbase = 2.68 m`, `trackWidth = 1.58 m`, `frontWeightDistribution = 0.52`, `wheelRadius = 0.32 m`, `wheelInertia = 1.15 kg·m²`, `frontSpringStiffness = 35000 N/m`, `rearSpringStiffness = 32500 N/m`, `idleRpm = 850`, `maxRpm = 7000`, 6-speed + R + N gear ratios, `finalDriveRatio = 3.96`, `tireFrictionCoeff = 1.15`, `dragCoefficient = 0.31`, `frontalArea = 2.15 m²`).
- [`src/vehicle/WheelState.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/vehicle/WheelState.h): Per-wheel state (`WheelState`) storing raycast contact, suspension compression/velocity/forces, slip ratio $\kappa$, slip angle $\alpha$, longitudinal/lateral tire forces, angular velocity $\omega$, rotation angle, and drive/brake torques.
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
- [`src/renderer/ModelLoaderGLB.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ModelLoaderGLB.h) & [`ModelLoaderGLB.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ModelLoaderGLB.cpp): Single-header `cgltf`-based binary glTF 2.0 / GLB loader. Parses multi-node hierarchies into OpenGL VAOs, maps nodes to semantic vehicle roles (Chassis, 4 independent wheel assemblies, calipers, steering wheel), parses PBR metallic/roughness & transparent glass materials, reads optional JSON manifests (`model/BMW_M4_manifest.json`), and automatically computes wheel rotation pivots.
- [`src/renderer/ShadowMapFBO.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ShadowMapFBO.h) & [`ShadowMapFBO.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/ShadowMapFBO.cpp): `2048x2048` depth framebuffer for directional sun shadows.
- [`src/renderer/TrackRendererGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/TrackRendererGL.h) & [`TrackRendererGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/TrackRendererGL.cpp): Procedural grid/surface textures, Proving Ground meshes, 25m distance marker posts, and handling ring.
- [`src/renderer/VehicleRendererGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/VehicleRendererGL.h) & [`VehicleRendererGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/VehicleRendererGL.cpp): Master vehicle renderer with hot-swapping between the Blender model (`BMW M4 Coupe`, 743,509 triangles across 58 chassis nodes + 4 articulated wheel assemblies) and the procedural sports coupe. Supports 2-pass rendering (opaque + transparent glass), reactive brake/reverse lights, suspension struts, and a 3D visual alignment debug overlay (`renderVisualDebug`).
- [`src/renderer/DebugOverlayGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/DebugOverlayGL.h) & [`DebugOverlayGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/DebugOverlayGL.cpp): 3D force vector gizmos (normal load $F_z$, longitudinal $F_x$, lateral $F_y$, velocity vector, and CoM marker) + rear-wheel dynamic skidmarks.
- [`src/renderer/CameraControllerGL.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/CameraControllerGL.h) & [`CameraControllerGL.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/renderer/CameraControllerGL.cpp): 4 camera modes (`Follow`, `Orbit`, `Suspension`, `Hood`).
- [`src/ui/ImGuiDashboard.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/ui/ImGuiDashboard.h) & [`ImGuiDashboard.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/ui/ImGuiDashboard.cpp): Top bar (zone presets `1..7`, model toggle, camera/gizmo/telemetry toggles, physics verifier button), left live tuning panel (model switcher, alignment debug checkbox, 12 real-time sliders + drivetrain layout selector), right telemetry panel (force balance summary, 4-wheel table, 4-band real-time oscilloscope), and bottom instrument cluster (digital speedometer, RPM bar, pedal bars, G-G diagram).
- [`src/input/InputControllerGLFW.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/InputControllerGLFW.h) & [`InputControllerGLFW.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/input/InputControllerGLFW.cpp): Progressive keyboard + baseline-calibrated gamepad input handler (including `B` key to toggle 3D model, `V` key to toggle alignment debug).
- [`src/app/Application.h`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/app/Application.h), [`Application.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/app/Application.cpp), [`main.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/src/app/main.cpp): Lifecycle manager and CLI entry point (`--smoke-test`, `--diag`).
- [`tests/native_validation_tests.cpp`](file:///d:/%23%20My%20Folder/My%20Work/Visual%20Studio%20script/WebApplications/TorqueLab/tests/native_validation_tests.cpp): 21 automated test suites (`Unit 1–8`, `Validation Tests A–J`, `Controlled Tests 1–3`).

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

## 6. Verified Physical Benchmarks (`24 / 24` Suites Passing)

Any future code change must keep all 24 suites in `.\build\Release\TorqueLabNativeTests.exe` green:

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

## 7. Vehicle Configuration, Materials & Collision System Architecture

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
