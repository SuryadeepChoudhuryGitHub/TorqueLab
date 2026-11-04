# TORQUE LAB NATIVE (Project Torque)
### Realistic 3D Vehicle Physics Simulator & Engineering Laboratory — Native C++20 / OpenGL 3.3 / Bullet Physics / Dear ImGui

**Where physical laws govern every motion.**

**Torque Lab Native** is a high-performance Windows desktop 6-DOF vehicle dynamics simulator and interactive engineering laboratory built with **C++20**, **OpenGL 3.3 Core Profile**, **GLSL 330**, **GLFW 3.4**, **GLAD**, **Bullet Physics (`btDiscreteDynamicsWorld`)**, **Dear ImGui**, and **CMake**.

Every vehicle behavior — acceleration, braking, cornering, pitch (brake dive and acceleration squat), body roll, load transfer, tire wheelspin, brake lockup, understeer/oversteer, surface grip transitions, airborne ballistics, obstacle collisions, and controller force feedback — emerges strictly from coherent physical laws (forces, torques, mass, rotational inertia, suspension spring-dampers, and slip-based friction circles).

---

## 1. Key Capabilities & Architecture

- **6-DOF Rigid-Body Dynamics & Suspension (120 Hz Fixed Timestep):**
  - Independent 4-wheel suspension with Hooke springs ($F_s = -k \Delta x$), asymmetric bound/rebound dampers ($c_{\text{bound}} \neq c_{\text{rebound}}$), progressive quadratic bump stops, and front/rear anti-roll bars.
  - Pacejka Magic Formula tire model with load-sensitive friction coefficient $\mu(F_z)$ and combined-slip friction circle saturation ($\sqrt{F_x^2 + F_y^2} \le \mu F_z$).
  - Piecewise-linear engine torque curve, automatic/manual transmission, launch clutch slip engagement, soft rev limiter at 7,000 RPM, RWD/FWD/AWD drive layout distribution, and speed-sensing viscous Limited-Slip Differential (LSD).
  - Quadratic aerodynamic drag ($F_d = \frac{1}{2}\rho C_d A v^2$) and aerodynamic downforce ($F_L = \frac{1}{2}\rho C_L A v^2$).
  - 6 distinct road surfaces: Dry Asphalt ($\mu=1.00$), Wet Asphalt ($\mu=0.68$), Gravel ($\mu=0.54$), Ice ($\mu=0.18$), Grass ($\mu=0.42$), and Rumble Strips ($\mu=0.88$).

- **Full glTF 2.0 Metallic-Roughness PBR & HDR Environment Lighting:**
  - Integrated `cgltf`-based binary glTF 2.0 / GLB loader parsing hierarchical scene graphs, node transformations, wheel rotation pivots, and authentic PBR materials.
  - Multi-car garage with interactive top-bar and panel dropdown selectors:
    - **BMW M4 Coupe:** High-poly modern twin-turbo sports coupe (743,509 triangles across 58 chassis nodes + 4 articulated wheel assemblies, $550\text{ N}\cdot\text{m}$, 6-speed RWD).
    - **1970 Dodge Challenger R/T:** Authentic classic American muscle car (1,915,112 triangles across 162 chassis nodes + 4 independent wheel assemblies, 426 Hemi V8, $664\text{ N}\cdot\text{m}$, 4-speed manual gearbox, Dana 60 Sure-Grip LSD, authentic launch squat).
    - **Procedural Sports Coupe:** Lightweight procedural baseline coupe with full parameter customizability.
  - **Cook-Torrance GGX Microfacet BRDF:** Physical metallic-roughness lighting with Trowbridge-Reitz GGX distribution ($D$), Smith-Schlick geometric shadowing ($G$), Fresnel-Schlick reflectivity ($F$), and energy conservation ($k_D + k_S \le 1.0$).
  - **Complete Multi-Channel Texture Extraction:**
    - Unit 0: **Base Color / Albedo** (`GL_SRGB8_ALPHA8`, sRGB color space)
    - Unit 1: **Directional Shadow Map** (`GL_DEPTH_COMPONENT24`, depth)
    - Unit 2: **HDR Environment Panoramic Map** (`GL_RGBA8`, IBL)
    - Unit 3: **Tangent-Space Normal Map** (`GL_RGBA8`, Linear space) with `normalScale`
    - Unit 4: **Metallic-Roughness Map** (`GL_RGBA8`, Linear space; Green = Roughness, Blue = Metalness)
    - Unit 5: **Ambient Occlusion Map** (`GL_RGBA8`, Linear space; Red = AO) with `occlusionStrength`
    - Unit 6: **Emissive Map** (`GL_SRGB8_ALPHA8`, sRGB color space)
  - **MikkTSpace-Compatible Orthonormal Tangents:** Automatic smooth tangent ($\vec{T}$) and bitangent handedness sign ($tw \in \{-1.0, 1.0\}$) generation across indexed triangle meshes for authentic bump details.
  - **Automotive Studio & Sky HDR Environment Map (IBL):** Procedural panoramic sky dome with sun corona and 3 overhead studio softbox light strips; trilinear roughness mipmaps translate surface roughness directly into physical specular blur.
  - **ACES Filmic Tone Mapping & Gamma 2.2:** Linear lighting workflow with photographic tone reproduction.
  - **Real-Time Shading Diagnostic Modes & Material Inspector:** Immediate inspection via Dear ImGui across 9 modes (Final PBR, Albedo, Metalness, Roughness, Shading Normals, AO, Emissive, Tangents, Environment Specular) with interactive live material breakdown tables.

- **Automatic Model-Derived Collision Pipeline & Compile-Time Auto-Baking:**
  - V-HACD volumetric decomposition and spatial topological clustering via `btConvexHullComputer`.
  - Automatically derives wheelbase, track width, wheel radius, and ground clearance directly from 3D model geometry.
  - Generates 10 watertight convex collision hulls (6 chassis pieces + 4 dedicated cylindrical wheel hulls) in Bullet's `btCompoundShape` aligned 1:1 with visual body nodes.
  - **Automated Compile-Time & Runtime Baking:** Integrated CMake post-build step (`TorqueLabNative --bake-collisions`) automatically bakes collision geometry whenever compiling. Missing or modified models (`.glb` newer than cached `.json`) are instantly detected via file timestamps and baked headlessly on pure CPU without requiring an OpenGL window.
  - **Dynamic Multi-Vehicle Auto-Discovery:** Dropping any new `.glb` into `model/` automatically discovers it, bakes physical collision hulls, and exposes it in the in-game car selection dropdown and controller cycling (`RB` / `B`).
  - Fast disk caching (`model/cache/<vehicle_id>_collision.json`) and Continuous Collision Detection (CCD) to prevent high-speed tunneling.

- **Full Controller Support Inspired by Forza Horizon:**
  - Comprehensive multi-gamepad input subsystem using GLFW and dynamic Windows XInput.
  - Analog trigger acceleration and braking with automatic reverse integration in Automatic mode.
  - Continuous analog steering with configurable deadzones, progressive sensitivity curves, and steering inversion.
  - **Resting Baseline Calibration Guard:** Automatically captures initial idle axis baselines on connection to neutralize virtual HID devices (such as Monect or vJoy) and eliminate phantom brake lockup.
  - Right-stick free-look look-around chase camera with heading damping.
  - Timed hold-to-recover vehicle reset action (`Y` / `Triangle`) preventing accidental resets.
  - Interactive button rebinding interface with automatic Xbox vs. PlayStation button glyph adaptivity.
  - Persistent JSON configuration in `settings/controller_config.json`.

- **Physics Event-Driven Controller Haptics (Strictly Zero RPM Vibration):**
  - **Total RPM Vibration Elimination:** Engine RPM, throttle position, engine audio, and smooth cruising produce strictly **0.0% vibration**.
  - **Collision Impact Feedback:** Severity scaled non-linearly from rigid-body velocity jumps ($|\Delta \vec{v}|$) and Bullet contact normal impulses. Contact lifetime filtering silences settled resting contact against walls.
  - **Lateral Directionality:** Collision contact points mapped to chassis coordinates weight the left vs. right vibration motors ($[-1.0 .. +1.0]$).
  - **Suspension Events:** Distinct transient pulses for airborne jump landings ($v_{\text{down}} > 0.6\text{ m/s}$), progressive bump-stop bottoming out ($w.\text{bumpStopForce} > 800\text{ N}$), sharp curb/bump strikes ($dL/dt < -2.2\text{ m/s}$), and underside chassis scrapes.
  - Independent low-frequency (heavy thuds) and high-frequency (sharp impacts/scrapes) motor routing with smooth cosine decay envelopes and collision deduplication cooldowns.

- **Interactive Engineering Dashboard (Dear ImGui):**
  - Live tuning sliders for mass, CoM height, spring stiffness, damping, anti-roll bars, tire grip, and brake bias.
  - Real-time 4-wheel telemetry table and 4-band oscilloscope (G-forces, suspension travel, tire slip, engine RPM).
  - Digital instrument cluster with speedometer, RPM bar, pedal meters, G-G diagram, and transmission mode selector.
  - 3D force vector gizmos (normal load $F_z$, traction $F_x$, cornering $F_y$, velocity vector, and CoM marker).

---

## 2. How to Build & Run (Windows x64)

### Quick Run
The Release binaries and assets are located in [`build/Release/`](build/Release):

```powershell
# 1. Launch the interactive 3D desktop simulator (1440x900 window)
.\build\Release\TorqueLabNative.exe

# 2. Run the 28-suite automated unit, validation & physics test harness
.\build\Release\TorqueLabNativeTests.exe

# 3. Run full-pipeline acceleration telemetry table + 600-frame live OpenGL timing diagnostics
.\build\Release\TorqueLabNative.exe --diag

# 4. Run headless OpenGL + Bullet smoke test (quick sanity check)
.\build\Release\TorqueLabNative.exe --smoke-test
```

### Build from Source (Visual Studio 2022 / CMake)
**Prerequisites:**
- **CMake** `3.20+`
- **Visual Studio 2022 or 2026** with **Desktop development with C++** (MSVC x64)
- **OpenGL 3.3+** capable GPU and display driver
- *(All third-party libraries — `GLFW 3.4`, `Bullet Physics 3.25`, `Dear ImGui v1.91.5`, `cgltf`, and `V-HACD` — are managed automatically via CMake `FetchContent` or vendored in `third_party/`).*

```powershell
# 1. Configure CMake build directory
cmake -B build -A x64

# 2. Compile both the application and the automated test runner in Release mode
cmake --build build --config Release --target TorqueLabNative TorqueLabNativeTests
```

---

## 3. Controls & Proving Ground Layout

### Controller Controls (Forza Horizon Default Layout)

| Action | Xbox Controller | PlayStation Controller | Behavior / Notes |
| :--- | :--- | :--- | :--- |
| **Accelerate** | **`RT`** | **`R2`** | Analog trigger ($0.0 \to 1.0$) with deadzone & curve |
| **Brake / Reverse** | **`LT`** | **`L2`** | Analog trigger ($0.0 \to 1.0$); auto-reverses when stopped in `AUTO` |
| **Steer Left / Right** | **Left Stick (X)** | **Left Stick (X)** | Analog horizontal axis with speed-sensitive Ackermann limits |
| **Handbrake** | **`A`** | **`Cross`** | Locks rear wheels ($3,400\text{ N}\cdot\text{m}$) |
| **Shift Up** | **`B`** | **`Circle`** | Upshift in `MANUAL` transmission mode |
| **Shift Down** | **`X`** | **`Square`** | Downshift in `MANUAL` transmission mode |
| **Reset Vehicle** | **`Y` (Hold)** | **`Triangle` (Hold)** | Hold for $0.45\text{ s}$ to recover vehicle to equilibrium |
| **Look Around** | **Right Stick** | **Right Stick** | $360^\circ$ analog free-look camera orbit |
| **Change Camera** | **`R3` (Stick Click)** | **`R3` (Stick Click)** | Cycles Follow $\to$ Orbit $\to$ Suspension Rig $\to$ Driver Hood |
| **Transmission Mode**| **`LB`** | **`L1`** | Toggles Automatic vs. Manual transmission |
| **Switch Car Model** | **`RB`** | **`R1`** | Cycles BMW M4 $\to$ Dodge Challenger $\to$ Procedural Coupe |
| **Pause Simulation** | **`Start` / `Menu`** | **`Options`** | Pauses physics; routes gamepad to ImGui menus |
| **Visual Alignment** | **`Back` / `View`** | **`Share` / `Touchpad`**| Toggles 3D collision wireframe & visual alignment gizmos |
| **Cycle Proving Zone**| **`D-Pad Left/Right`**| **`D-Pad Left/Right`** | Teleports to previous / next Proving Ground test zone |
| **Toggle Telemetry** | **`D-Pad Up`** | **`D-Pad Up`** | Toggles telemetry cards and 4-band oscilloscope |
| **Toggle 3D Gizmos** | **`D-Pad Down`** | **`D-Pad Down`** | Toggles 3D force arrows, CoM marker, and friction circles |

### Keyboard & Mouse Controls

| Key / Input | Action |
| :--- | :--- |
| **`W` / `Up Arrow`** | Progressive Throttle |
| **`S` / `Down Arrow`** | Service Brakes (automatically shifts to Reverse when stopped in `AUTO` mode) |
| **`A` / `Left Arrow`** | Steer Left |
| **`D` / `Right Arrow`** | Steer Right |
| **`Space`** | Rear-Wheel Handbrake |
| **`B`** | Cycle Car Model (BMW M4 $\to$ Dodge Challenger $\to$ Procedural Coupe) |
| **`V`** | Toggle Visual Alignment Debug Gizmos |
| **`M`** | Toggle Automatic / Manual Transmission |
| **`E` / `Q`** | Manual Upshift / Downshift |
| **`R`** | Reset Vehicle to clean equilibrium state |
| **`C`** | Cycle Camera (`Chase Follow` $\to$ `Orbit Inspection` $\to$ `Suspension Rig` $\to$ `Driver Hood`) |
| **`P` / `Escape`** | Pause / Resume Physics Simulation |
| **`T`** | Toggle Telemetry Table & Real-Time Oscilloscope |
| **`G`** | Toggle 3D Force Vectors, Raycasts & Friction Circles |
| **`1` – `7`** | Instant Teleport to Proving Ground Zones 1–7 |
| **Left Mouse Drag + Scroll** | Orbit Camera Azimuth / Elevation & Zoom (in `Orbit Inspection` mode) |

### Proving Ground Test Zones (`1`–`7` or Top Bar)
1. **1 Main Straight (`1`):** High-speed acceleration & braking runway on Dry Asphalt ($\mu = 1.00$) with $25\text{ m}$ distance posts.
2. **2 Handling Circle (`2`):** North steady-state cornering skidpad ($R = 28\text{ m}$) for body roll and understeer/oversteer analysis.
3. **3 Wet Skidpad (`3`):** Reduced-grip Wet Asphalt ($\mu = 0.68$) braking & cornering zone.
4. **4 Rally & Ice (`4`):** Loose Gravel rally lane ($\mu = 0.54$) and low-grip Polished Ice sheet ($\mu = 0.18$).
5. **5 Bump Course (`5`):** Rhythmic suspension compression, rebound & damper dissipation course.
6. **6 Elevation Ramp (`6`):** Inclined launch ramp and landing ramp for airborne trajectory & suspension bump-stop testing.
7. **7 Slalom & Obstacles (`7`):** Slalom bollards and perimeter barrier walls for transient weight-transfer & collision momentum testing.

---

## 4. Repository Structure

```text
TorqueLab/
├── CMakeLists.txt                    # Native C++20 build definition (Bullet, GLFW, ImGui, V-HACD)
├── CMakePresets.json                 # MSVC x64 build presets
├── README.md                         # Project documentation and engineering guide
├── GEMINI.md                         # Single-source-of-truth project memory & physical conventions
├── model/                            # Vehicle 3D assets & collision cache
│   ├── BMW_M4.glb                    # Primary 743k-tri high-poly PBR model
│   ├── BMW_M4_manifest.json          # Node hierarchy & semantic role manifest
│   ├── textures/                     # 2048x2048 PBR livery and emblem textures
│   └── cache/                        # Cached volumetric collision hulls (*_collision.json)
├── settings/
│   └── controller_config.json        # Persistent controller deadzones, sensitivities & rebinds
├── shaders/                          # GLSL 330 Core Shaders
│   ├── shadow_depth.vert/.frag       # Directional sun shadow depth pass
│   ├── phong_shadow.vert/.frag       # Blinn-Phong + PCF shadows + glass transparency + fog
│   └── debug_line.vert/.frag         # 3D force vector gizmos & collision wireframes
├── third_party/
│   ├── glad/                         # OpenGL 3.3 Core Profile loader
│   ├── cgltf/                        # Single-header glTF 2.0 / GLB parser
│   └── vhacd/                        # V-HACD volumetric convex decomposition library
├── src/
│   ├── app/                          # Lifecycle & main loop
│   │   ├── main.cpp                  # CLI entry point (--smoke-test, --diag)
│   │   └── Application.h/.cpp        # GLFW window, 120 Hz accumulator loop, 2-pass renderer
│   ├── physics/                      # Core Math, Bullet World & Collision Generation
│   │   ├── Math3D.h                  # Vec3, Quat, Mat4, VehicleBasis & Euler angle extraction
│   │   ├── CollisionMeshGeometry.h   # Raw mesh geometry & convex hull definitions
│   │   ├── VehicleCollisionGenerator.h/.cpp # V-HACD decomposition & vehicle parameter derivation
│   │   ├── SurfaceSystem.h/.cpp      # Road surface registry (6 surfaces) & friction coefficients
│   │   └── PhysicsWorld.h/.cpp       # Bullet world, Proving Ground track, 7 spawn presets
│   ├── vehicle/                      # 6-DOF Vehicle Subsystems
│   │   ├── VehicleConfig.h           # Physical vehicle configuration struct
│   │   ├── WheelState.h              # Per-wheel contact, suspension, slip, forces & torques
│   │   ├── SuspensionSystem.h/.cpp   # Spring-damper, bump stop & anti-roll bar mechanics
│   │   ├── TireModel.h/.cpp          # Pacejka Magic Formula, load sensitivity & friction circle
│   │   ├── SteeringSystem.h/.cpp     # Speed-sensitive steering & Ackermann geometry
│   │   ├── Drivetrain.h/.cpp         # Torque curve, rev limiter, 6-speed gearbox & viscous LSD
│   │   ├── BrakeSystem.h/.cpp        # Front/rear service brakes & rear mechanical handbrake
│   │   ├── Aerodynamics.h/.cpp       # Quadratic drag, aerodynamic downforce & rolling resistance
│   │   ├── VehicleBody.h/.cpp        # Rigid body wrapper, collision impulse detection & scrapes
│   │   └── VehicleDynamics.h/.cpp    # Master 120 Hz orchestrator & suspension haptic event detection
│   ├── renderer/                     # OpenGL 3.3 Rendering Subsystem
│   │   ├── Shader.h/.cpp             # GLSL shader compilation & uniform cache
│   │   ├── Mesh.h/.cpp               # Procedural primitives (boxes, cylinders, rings)
│   │   ├── Texture2D.h/.cpp          # Procedural surface grain textures & PNG loaders
│   │   ├── ShadowMapFBO.h/.cpp       # 2048x2048 depth framebuffer for sun shadows
│   │   ├── ModelLoaderGLB.h/.cpp     # glTF 2.0 parser & semantic vehicle hierarchy binder
│   │   ├── VehicleRendererGL.h/.cpp  # Master vehicle renderer (BMW M4 / Procedural Coupe)
│   │   ├── TrackRendererGL.h/.cpp    # Track element meshes, distance markers & surfaces
│   │   ├── DebugOverlayGL.h/.cpp     # 3D force gizmos, CoM marker & skidmarks
│   │   └── CameraControllerGL.h/.cpp # Chase Follow, Orbit, Suspension & Hood cameras
│   ├── input/                        # Controller & Input Subsystem
│   │   ├── ControllerConfig.h/.cpp   # Logical actions, Forza mappings, glyphs & JSON persistence
│   │   ├── VibrationManager.h/.cpp   # Event-driven XInput haptics (collisions, suspension, scrapes)
│   │   └── InputControllerGLFW.h/.cpp# Gamepad polling, baseline calibration & rebinding
│   └── ui/
│       └── ImGuiDashboard.h/.cpp     # Top bar, live tuning panel, telemetry, cluster & oscilloscope
└── tests/
    └── native_validation_tests.cpp   # 27 automated test suites (Unit 1–14, Tests A–J, Controlled 1–3)
```

---

## 5. Verification & Benchmark Summary (`28 / 28` Suites Passing)

All physical behaviors are continuously validated via the automated C++ test harness (`TorqueLabNativeTests.exe`):

| Test Suite | Result | Verified Physical Behavior & Benchmark Numbers |
| :--- | :---: | :--- |
| **Unit 1–8** | **PASS** | Suspension signs, coordinate transforms, Pacejka friction circles, slip kinematics, engine torque curve & LSD, surface hierarchy, Ackermann steering, quadratic aerodynamic drag. |
| **Unit 9–12** | **PASS** | Model-derived collision pipeline, dynamic hull stability, continuous collision detection (CCD), collision lifecycle, and vehicle switching integrity. |
| **Unit 13** | **PASS** | Forza-inspired controller mapping, trigger normalization, steering deadzone/sensitivity curves, and JSON serialization roundtrip. |
| **Unit 14** | **PASS** | **Physics-Driven Haptics & RPM Vibration Elimination:** Stationary engine redline at 7,000 RPM yields strictly 0.0% vibration; high-speed cruising yields 0.0% vibration; collision scaling, lateral motor bias, jump landing thuds, and bump-stop jolts verified. |
| **Unit 15** | **PASS** | **Multi-Vehicle Presets & Muscle Car Tuning:** 1970 Dodge Challenger R/T 426 Hemi specs ($1,680\text{ kg}$, $2.794\text{ m}$ wheelbase, $664\text{ N}\cdot\text{m}$ peak torque, 4-speed manual gearbox, Dana 60 LSD) and zero-leak 3-way vehicle switching verified. |
| **Test A (Static Equilibrium)** | **PASS** | Settled tire load $\sum F_z = 14,715.0\text{ N}$ ($0.000\%$ error vs $mg = 14,715.0\text{ N}$); ride height $= 0.4910\text{ m}$; suspension compression $= 0.1093\text{ m}$. |
| **Test B (Acceleration & Squat)** | **PASS** | At $t = 2.0\text{ s}$: $v = 10.48\text{ m/s}$ ($37.7\text{ km/h}$), $a_x = +5.27\text{ m/s}^2$, rear squat pitch $= +0.800^\circ$, rear load transfer $\Delta F_{z,\text{rear}} = +1315\text{ N}$. |
| **Test C (Braking & Dive)** | **PASS** | Service braking from $20\text{ m/s}$: $a_x = -9.68\text{ m/s}^2$, front dive pitch $= -1.614^\circ$, $F_{z,\text{front}} = 9,985\text{ N}$ vs $F_{z,\text{rear}} = 4,696\text{ N}$. |
| **Test D (Cornering & Roll)** | **PASS** | Left turn roll $= +2.31^\circ$ ($a_y = +9.49\text{ m/s}^2$); Right turn roll $= -2.28^\circ$ ($a_y = -9.36\text{ m/s}^2$). |
| **Test E (Surface Hierarchy)** | **PASS** | Dry asphalt launch $v = 5.94\text{ m/s}$, stopping distance $= 11.49\text{ m}$ vs Ice launch $v = 1.23\text{ m/s}$, stopping distance $= 52.01\text{ m}$. |
| **Test F (Speed Bump Response)** | **PASS** | Equilibrium compression $0.109\text{ m} \to$ Peak front $0.191\text{ m}$, peak pitch $2.88^\circ \to$ settles cleanly to $0.07^\circ$. |
| **Test G (Parameter Sensitivity)** | **PASS** | Stiffer springs reduce compression by $37.5\%$; high CoM increases body roll from $1.78^\circ$ to $3.58^\circ$ ($+101\%$). |
| **Test H (Collision Transfer)** | **PASS** | Post-crash vehicle stops cleanly at barrier ($Z = 11.15\text{ m}$) without tunneling; recovery reset restores clean state. |
| **Test I (Timestep Independence)**| **PASS** | Fixed $1/120\text{ s}$ accumulator produces identical trajectories at 30 FPS, 60 FPS, and 120 FPS ($Z = 9.025\text{ m}$, $V = 9.672\text{ m/s}$). |
| **Test J (Coasting Deceleration)**| **PASS** | Initial speed $25.00\text{ m/s} \to 24.13\text{ m/s}$ over $3.0\text{ s}$ matching analytical aerodynamic + rolling resistance loss. |
| **Controlled 1–3** | **PASS** | Constant force $a = F/m$ verified ($3.869\text{ m/s}^2$ vs expected $3.867\text{ m/s}^2$); zero-resistance baseline; scale sanity ($d = v \cdot t = 50.005\text{ m}$ over $2.0\text{ s}$). |

---

## 6. License & Credits

- **Physics & Simulation Engine:** Built by the Project Torque team using native C++20 and Bullet Physics.
- **Rendering & UI:** OpenGL 3.3 Core Profile, GLFW 3.4, GLAD, and Dear ImGui.
- **Asset Credits:** BMW M4 Coupe 3D model and textures under editorial/educational simulation license.
