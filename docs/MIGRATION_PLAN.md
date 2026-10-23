# Torque Lab Native — Staged Migration Plan (`MIGRATION_PLAN.md`)

**Objective:** Execute a complete, verified migration of Torque Lab from TypeScript/Three.js/Rapier3D to Native C++20/OpenGL 3.3/Bullet Physics/Dear ImGui on Windows.

---

## 1. Staged Implementation Order

### Phase 1 — Source Audit & Engineering Baseline *(Completed)*
- Inspect all 23 TypeScript source and test files.
- Produce `docs/MIGRATION_AUDIT.md`, `docs/ARCHITECTURE.md`, `docs/FEATURE_PARITY_MATRIX.md`, `docs/PHYSICS_BASELINE.md`, and `docs/MIGRATION_PLAN.md`.

### Phase 2 — Native Build System & Third-Party Dependencies
- Generate self-contained OpenGL 3.3 Core loader (`third_party/glad/include/glad/glad.h`, `third_party/glad/include/KHR/khrplatform.h`, `third_party/glad/src/glad.c`).
- Create `CMakeLists.txt` and `CMakePresets.json` fetching:
  - **GLFW 3.4** (`glfw/glfw`)
  - **Bullet Physics 3.25** (`bulletphysics/bullet3` — `LinearMath`, `BulletCollision`, `BulletDynamics` only, unit tests/demos disabled for fast build)
  - **Dear ImGui v1.91.8** (`ocornut/imgui` with `imgui_impl_glfw.cpp` and `imgui_impl_opengl3.cpp`)
- Define three CMake targets:
  1. `TorqueLabPhysics` (static library containing `src/physics/*` and `src/vehicle/*`)
  2. `TorqueLabNative` (interactive OpenGL + Dear ImGui desktop application)
  3. `TorqueLabNativeTests` (headless C++ automated test & comparative verification executable)

### Phase 3 — Core Physics & Vehicle Dynamics Migration (`TorqueLabPhysics`)
- Implement `src/physics/Math3D.h` (`Vec3`, `Quat`, `Mat4`, right-handed basis, Euler extraction, slerp).
- Implement `src/vehicle/VehicleConfig.h`, `src/physics/SurfaceSystem.h/.cpp`, and `src/vehicle/WheelState.h`.
- Implement all vehicle dynamics subsystems with 1:1 equations:
  - `src/vehicle/SuspensionSystem.h/.cpp`
  - `src/vehicle/TireModel.h/.cpp`
  - `src/vehicle/SteeringSystem.h/.cpp`
  - `src/vehicle/Drivetrain.h/.cpp`
  - `src/vehicle/BrakeSystem.h/.cpp`
  - `src/vehicle/Aerodynamics.h/.cpp`
- Implement `src/vehicle/VehicleBody.h/.cpp` wrapping Bullet `btRigidBody` + `btCompoundShape` with exact Center-of-Mass offset transformation between Geometric Chassis Origin and Bullet CoM frame.
- Implement `src/vehicle/VehicleDynamics.h/.cpp` and `src/physics/PhysicsWorld.h/.cpp` (including custom raycast callback ignoring the vehicle's own chassis body, Proving Ground track builder, and 7 spawn presets).

### Phase 4 — OpenGL 3.3 Core Renderer & GLSL Shaders
- Author GLSL 330 core shaders in `shaders/`:
  - `shadow_depth.vert`, `shadow_depth.frag`
  - `phong_shadow.vert`, `phong_shadow.frag`
  - `debug_line.vert`, `debug_line.frag`
- Implement OpenGL renderer modules in `src/renderer/`:
  - `Shader.h/.cpp`, `Mesh.h/.cpp` (procedural Box, Cylinder, Torus, Sphere, Plane, Ring), `Texture2D.h/.cpp` (procedural 256×256 surface textures), `ShadowMapFBO.h/.cpp`.
  - `VehicleRendererGL.h/.cpp`, `TrackRendererGL.h/.cpp`, `DebugOverlayGL.h/.cpp`, `CameraControllerGL.h/.cpp`.

### Phase 5 & 6 — GLFW Input, Dear ImGui HUD, Telemetry, Oscilloscope & Live Tuning
- Implement `src/input/InputControllerGLFW.h/.cpp` (progressive pedal ramping, keyboard shortcuts, gamepad polling, and ImGui focus awareness).
- Implement `src/ui/ImGuiDashboard.h/.cpp` (top toolbar, 7 zone presets, quality selector, self-test banner, left controls guide, live tuning lab with 12 sliders + drivetrain combo + defaults reset, right 4-wheel telemetry table + 4-band real-time oscilloscope, and bottom instrument cluster with 2D G-G diagram).
- Implement `src/app/Application.h/.cpp` and `src/app/main.cpp` (supporting `--smoke-test` for automated OpenGL/ImGui frame verification as well as normal interactive desktop execution).

### Phase 7 & 8 — Automated Verification, Build Execution & Final Documentation
- Implement `tests/native_validation_tests.cpp` running all 8 Unit Test suites and 10 Physical Validation Scenarios (`Tests A–J`).
- Configure and build `TorqueLabNative` and `TorqueLabNativeTests` with CMake + MSVC/Ninja.
- Run `TorqueLabNativeTests` and `TorqueLabNative --smoke-test`, verify zero errors, record comparative numerical outputs in `docs/TEST_RESULTS.md`, and update `README.md`.

---

## 2. Technical Risks & Mitigations

| Risk | Impact | Mitigation Strategy |
| :--- | :---: | :--- |
| **Bullet `btRigidBody` origin is always at the CoM**, whereas Rapier kept the body origin at the geometric chassis origin and offset the CoM internally. | Incorrect suspension mount heights or pitch/roll lever arms if untreated. | Wrap chassis colliders in a `btCompoundShape` shifted by $-\mathbf{r}_{\text{CoM}}$, and provide `VehicleBody::getPosition()` / `resetPose()` in Geometric Chassis Origin coordinates ($\mathbf{p}_{\text{origin}} = \mathbf{p}_{\text{CoM}} - \mathbf{R}\mathbf{r}_{\text{CoM}}$). |
| **Bullet raycast hitting the vehicle's own chassis collider** when casting downward from suspension mounts. | False maximum suspension compression. | Implement a custom `btCollisionWorld::ClosestRayResultCallback` subclass (`ExcludeVehicleRayResultCallback`) that skips `vehicleBody->getRigidBody()`. |
| **Bullet contact manifold collection** differs from Rapier's `contactPairsWith`. | Missing collision contact markers in `DebugOverlayGL` / telemetry. | Iterate `dispatcher->getNumManifolds()` after `stepSimulation`, filter manifolds involving `vehicleBody`, and record active world contact points with distance $\le 0.02\text{ m}$. |
| **Working directory differences** when launching `TorqueLabNative.exe` from project root vs `build/` directory. | Shader files (`shaders/*.vert/.frag`) not found at runtime. | Implement multi-path shader resolution in `Shader.cpp` checking `./shaders/`, `../shaders/`, `../../shaders/`, and the executable directory. |
