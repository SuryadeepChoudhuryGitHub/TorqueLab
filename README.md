# TORQUE LAB NATIVE (Project Torque)
### Realistic 3D Vehicle Physics Simulator — Native C++20 / OpenGL 3.3 / Bullet Physics / Dear ImGui

**Where physics drives everything.**

**Torque Lab Native** is a native Windows desktop 6-DOF vehicle dynamics simulator and interactive engineering laboratory built with **C++20**, **OpenGL 3.3+ Core Profile**, **GLSL 330**, **GLFW**, **GLAD**, **Bullet Physics**, **Dear ImGui**, and **CMake**.

Every aspect of vehicle motion — acceleration, braking, cornering, pitch (brake dive & acceleration squat), body roll, load transfer, wheelspin, brake lockup, understeer/oversteer, surface grip transitions, airborne ballistics, and obstacle collisions — emerges from physical forces, torques, mass, rotational inertia, suspension spring-dampers, and slip-based tire friction circles.

---

## 1. How to Run & Build (Windows x64)

### Run Immediately (Already Built)
The Release binaries and GLSL shaders are already compiled in [`build/Release/`](build/Release):

```powershell
# 1. Launch the interactive 3D OpenGL + Dear ImGui desktop simulator
.\build\Release\TorqueLabNative.exe

# 2. Run the automated 18-suite C++ physics unit & validation test harness
.\build\Release\TorqueLabNativeTests.exe

# 3. Run the automated OpenGL + Bullet + Dear ImGui smoke test
.\build\Release\TorqueLabNative.exe --smoke-test
```

### Rebuild from Source (CMake + Visual Studio)
**Prerequisites:**
- **CMake** `3.20+`
- **Visual Studio 2022 or 2026** with the **Desktop development with C++** (MSVC x64) workload
- **OpenGL 3.3+** capable GPU / driver
- *(All third-party libraries — `GLFW 3.4`, `Bullet Physics 3.25`, `Dear ImGui v1.91.5` — are fetched automatically via CMake `FetchContent`, and `GLAD` OpenGL 3.3 Core is vendored in `third_party/glad/`).*

```powershell
# 1. Configure CMake build directory (Visual Studio x64)
cmake -B build -A x64

# 2. Build both the desktop simulator and the automated C++ test runner in Release mode
cmake --build build --config Release --target TorqueLabNative TorqueLabNativeTests
```

---

## 2. Repository Structure

```text
TorqueLab/
├── CMakeLists.txt                    # Native C++20 build configuration (GLFW, GLAD, Bullet, Dear ImGui)
├── CMakePresets.json                 # Windows MSVC x64 Debug/Release presets
├── README.md                         # Build, run, controls & architecture guide
├── docs/                             # Engineering & Migration Documentation
│   ├── MIGRATION_AUDIT.md            # Complete inventory of source functionality & subsystem audit
│   ├── ARCHITECTURE.md               # Native C++20 / OpenGL 3.3 / Bullet / ImGui architecture
│   ├── FEATURE_PARITY_MATRIX.md      # Feature-by-feature parity & verification matrix
│   ├── PHYSICS_BASELINE.md           # Governing SI equations, parameters, coordinate conventions & baselines
│   ├── MIGRATION_PLAN.md             # Staged migration plan, risk mitigations & completion criteria
│   └── TEST_RESULTS.md               # Executed C++ unit/validation test results (18/18 PASS)
├── shaders/                          # Native GLSL 330 Core Shaders
│   ├── shadow_depth.vert/.frag       # 2048x2048 directional sun depth shadow-map pass
│   ├── phong_shadow.vert/.frag       # Blinn-Phong + Fresnel + hemisphere ambient + PCF shadows + fog
│   └── debug_line.vert/.frag         # 3D world-space telemetry vectors, raycasts & friction circles
├── third_party/
│   └── glad/                         # Self-contained OpenGL 3.3 Core Profile loader (glad.h, glad.c)
├── src/
│   ├── app/                          # Native Application Lifecycle & Main Loop
│   │   ├── main.cpp                  # Native entry point & CLI flag handling (--smoke-test)
│   │   └── Application.h/.cpp        # GLFW window, 120 Hz physics accumulator, two-pass OpenGL & ImGui
│   ├── physics/                      # Native Core Math, Road Surfaces & Bullet World
│   │   ├── Math3D.h                  # 64-bit Vec3, Quat, Mat4, right-handed basis & Euler extraction
│   │   ├── SurfaceSystem.h/.cpp      # 6 driving surfaces (Dry/Wet Asphalt, Gravel, Ice, Grass, Rumble)
│   │   └── PhysicsWorld.h/.cpp       # Bullet btDiscreteDynamicsWorld, Proving Ground & 7 spawn presets
│   ├── vehicle/                      # Native 6-DOF Custom Vehicle Physics Engine
│   │   ├── VehicleConfig.h           # SI parameter struct, wheel mount offsets & equilibrium ride height
│   │   ├── WheelState.h              # Per-wheel contact, suspension, slip, tire force & torque state
│   │   ├── SuspensionSystem.h/.cpp   # Spring-damper (F_s = -kx - cv), quadratic bump stops & ARBs
│   │   ├── TireModel.h/.cpp          # Pacejka Magic Formula, load sensitivity & Friction Circle
│   │   ├── SteeringSystem.h/.cpp     # Speed-sensitive steering actuator & Ackermann wheel angles
│   │   ├── Drivetrain.h/.cpp         # 9-point torque curve, rev limiter, 6-speed + R gearbox & viscous LSD
│   │   ├── BrakeSystem.h/.cpp        # Front-biased service brakes & rear mechanical handbrake
│   │   ├── Aerodynamics.h/.cpp       # Quadratic air drag, downforce & rolling resistance
│   │   ├── VehicleBody.h/.cpp        # Bullet btRigidBody + btCompoundShape with dynamic CoM offset
│   │   └── VehicleDynamics.h/.cpp    # 120 Hz orchestrator, analytical OBB raycast & 8-substep wheel spin
│   ├── renderer/                     # Native OpenGL 3.3 Core Rendering Engine
│   │   ├── Shader.h/.cpp             # GLSL shader compiler & uniform manager
│   │   ├── Mesh.h/.cpp               # VAO/VBO/EBO procedural geometry primitives
│   │   ├── Texture2D.h/.cpp          # Procedural 256x256 RGB surface texture generator
│   │   ├── ShadowMapFBO.h/.cpp       # Depth framebuffer object for directional sun shadows
│   │   ├── VehicleRendererGL.h/.cpp  # Multi-part coupe body, cabin, spoiler, lights, calipers, struts & wheels
│   │   ├── TrackRendererGL.h/.cpp    # Proving Ground surfaces, ramps, bumps, bollards & distance markers
│   │   ├── DebugOverlayGL.h/.cpp     # 3D force arrows, CoM marker, suspension rays & friction circles
│   │   └── CameraControllerGL.h/.cpp # Chase Follow, Orbit Inspection, Suspension Rig & Driver Hood cameras
│   ├── input/
│   │   └── InputControllerGLFW.h/.cpp# Continuous keyboard pedal ramping, shortcuts & Gamepad polling
│   └── ui/
│       └── ImGuiDashboard.h/.cpp     # Dear ImGui Top Bar, Controls Card, Telemetry/Oscilloscope, Tuning & Cluster
└── tests/
    └── native_validation_tests.cpp   # 18 automated C++ unit & validation test suites (Unit 1–8, Tests A–J)
```

---

## 3. Controls & Proving Ground Zones

### Driving & Laboratory Controls
| Key / Input | Action |
| :--- | :--- |
| **`W` / `Up Arrow`** | Progressive Throttle |
| **`S` / `Down Arrow`** | Service Brakes (automatically shifts to Reverse when stopped in `AUTO` mode) |
| **`A` / `Left Arrow`** | Steer Left |
| **`D` / `Right Arrow`** | Steer Right |
| **`Space`** | Rear-Wheel Handbrake |
| **`M`** | Toggle Automatic / Manual Transmission |
| **`E` / `Q`** | Manual Upshift / Downshift |
| **`R`** | Reset Vehicle to clean equilibrium state |
| **`C`** | Cycle Camera (`Chase Follow` → `Orbit Inspection` → `Suspension Rig` → `Driver Hood`) |
| **`P` / `Escape`** | Pause / Resume Physics Simulation |
| **`T`** | Toggle Telemetry Table & 4-Band Real-Time Oscilloscope |
| **`G`** | Toggle 3D Force Vectors, Raycasts & Friction Circles |
| **`1` – `7`** | Instant Teleport to Proving Ground Test Zones 1–7 |
| **Left Mouse Drag + Scroll** | Orbit Camera Azimuth/Elevation & Zoom (in `Orbit Inspection` mode) |

### Proving Ground Test Zones (`1`–`7` or Top Bar Buttons)
1. **1 Main Straight (`1`):** High-speed acceleration & braking lane on Dry Asphalt ($\mu_{\text{mult}} = 1.00$) with $25\text{ m}$ distance markers.
2. **2 Handling Circle (`2`):** North steady-state cornering skidpad ($R = 28\text{ m}$) for body roll and understeer/oversteer testing.
3. **3 Wet Skidpad (`3`):** Reduced-grip Wet Asphalt ($\mu_{\text{mult}} = 0.65$) braking & cornering zone.
4. **4 Gravel & Ice (`4`):** Loose Gravel rally lane ($\mu_{\text{mult}} = 0.48$) and ultra-low-grip Polished Ice sheet ($\mu_{\text{mult}} = 0.20$).
5. **5 Bump Course (`5`):** Rhythmic suspension compression, rebound & damping test lane.
6. **6 Ramp Jump (`6`):** Inclined launch and landing ramps for airborne trajectory & bump-stop testing.
7. **7 Slalom (`7`):** Central slalom obstacle bollards and perimeter barriers for transient weight-transfer & collision testing.

---

## 4. Verification Summary (`18 / 18` Native Test Suites Passed)

See [`docs/TEST_RESULTS.md`](docs/TEST_RESULTS.md) for the complete numerical breakdown.

| Suite | Native C++ (`TorqueLabNativeTests.exe`) | Key Verified Physical Behavior |
| :--- | :---: | :--- |
| **Unit 1–8** | **8 / 8 PASS** | Spring/damper signs, right-handed transforms, Friction Circle $\sqrt{F_x^2+F_y^2}\le\mu_{\text{eff}}F_z$, load sensitivity, slip $\kappa/\alpha$, engine curve & LSD, surface hierarchy, Ackermann geometry, quadratic aero drag. |
| **Test A (Static Equilibrium)** | **PASS** (`Err = 0.000%`, $y = 0.4910\text{ m}$) | Settled vertical tire load $\sum F_z = 14,715.0\text{ N}$ matches $mg = 14,715.0\text{ N}$ within $0.000\%$. |
| **Test B (Acceleration & Squat)** | **PASS** ($10.54\text{ m/s}$, $\theta = +0.804^\circ$, $\Delta F_{z,R} = +1328\text{ N}$) | Longitudinal weight transfer compresses rear suspension and pitches nose up. |
| **Test C (Braking & Dive)** | **PASS** ($-9.82\text{ m/s}^2$, $\theta = -1.610^\circ$, $F_{z,F} = 9970\text{ N}$) | Service braking transfers load forward, compresses front suspension, and pitches nose down. |
| **Test D (Cornering & Roll)** | **PASS** ($\phi_L = +2.30^\circ$, $\phi_R = -2.27^\circ$) | Outward body roll and lateral load transfer reverse cleanly between left and right turns. |
| **Test E (Dry vs. Ice Grip)** | **PASS** ($d_{\text{dry}} = 11.36\text{ m}$ vs. $d_{\text{ice}} = 51.34\text{ m}$) | Ice exhibits $215\times$ higher launch wheelspin slip and $4.5\times$ longer stopping distance. |
| **Test F (Speed Bump Damping)** | **PASS** ($\theta_{\text{peak}} = 2.87^\circ \to 0.08^\circ$) | Wheel compression spikes over bump and damps cleanly back to equilibrium after clearing. |
| **Test G (Parameter Sensitivity)** | **PASS** ($c: 0.1093 \to 0.0683\text{ m}$; $\phi: 1.76^\circ \to 3.60^\circ$) | $1.6\times$ stiffer springs reduce compression by $37.5\%$; higher CoM increases roll by $104.5\%$. |
| **Test H (Collision & Reset)** | **PASS** ($z_{\text{stop}} = 11.12\text{ m}$, $v = -0.81\text{ m/s}$) | Barrier impact transfers momentum without tunneling; `resetVehicle()` restores exact clean state. |
| **Test I (Timestep Independence)** | **PASS** (`30/60/120 FPS` identical: $z = 9.427\text{ m}$) | Fixed `1/120 s` accumulator produces identical trajectories across render frame rates. |
| **Test J (Coasting Sanity)** | **PASS** ($25.00 \to 24.13\text{ m/s}$ over $3.0\text{ s}$) | Monotonic coasting deceleration matches analytical aerodynamic + rolling resistance loss. |
