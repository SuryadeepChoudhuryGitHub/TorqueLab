# Torque Lab Native — Feature Parity Matrix (`FEATURE_PARITY_MATRIX.md`)

This matrix maps every feature of the source browser application (`TypeScript / Three.js / Rapier3D`) to its native C++ implementation (`C++20 / OpenGL 3.3 / Bullet Physics / Dear ImGui`) and specifies its verification method.

---

## 1. Physics & Vehicle Dynamics Parity

| # | Feature / Subsystem | Web Source (`src/physics/`) | Native Implementation (`src/physics/`, `src/vehicle/`) | Parity Status | Verification Method |
| :-: | :--- | :--- | :--- | :---: | :--- |
| 1 | Right-handed coordinate system (`+X` Left, `+Y` Up, `+Z` Fwd) & Quaternion/Basis math | `math.ts` | `src/physics/Math3D.h` | **Exact 1:1** | Unit Test 2 (`Coordinate Transforms & Pitch/Roll Extraction`) |
| 2 | Fixed `120 Hz` ($\Delta t = 1/120\text{ s}$) timestep accumulator & render transform interpolation | `PhysicsWorld.ts`, `VehicleDynamics.ts` | `PhysicsWorld.cpp`, `VehicleDynamics.cpp` | **Exact 1:1** | Validation Test I (`Timestep Independence across 30/60/120 FPS`) |
| 3 | 6-DOF Rigid-Body Chassis with configurable mass, CoM height, front/rear weight bias, and inertia tensor | `VehicleBody.ts` | `VehicleBody.cpp` (`btRigidBody` + `btCompoundShape` CoM offset mapping) | **Exact 1:1** | Validation Tests A, B, C, D, G |
| 4 | 4-Wheel Independent Raycast Suspension with chassis-downward raycasts and contact normal projection | `VehicleDynamics.ts` | `VehicleDynamics.cpp` (`btCollisionWorld::rayTest` + `WheelFilterCallback`) | **Exact 1:1** | Validation Tests A & F |
| 5 | Linear Spring ($F_{\text{spring}} = -kx$) + Asymmetric Bound/Rebound Damper ($F_{\text{damper}} = -cv$) | `SuspensionSystem.ts` | `SuspensionSystem.cpp` | **Exact 1:1** | Unit Test 1 (`Spring & Damper Force Signs`) & Test A |
| 6 | Progressive Quadratic Bump Stops near minimum suspension length | `SuspensionSystem.ts` | `SuspensionSystem.cpp` (`computeBumpStopForce`) | **Exact 1:1** | Unit Test 1 & Validation Test F |
| 7 | Front & Rear Axle Anti-Roll Bars ($F_{\text{arb}} = k_{\text{arb}}(c_L - c_R)$) | `SuspensionSystem.ts` | `SuspensionSystem.cpp` (`computeAxleAntiRollForce`) | **Exact 1:1** | Unit Test 1 & Validation Test D |
| 8 | Tire Load Sensitivity ($\mu_{\text{eff}}(F_z)$) | `TireModel.ts` | `TireModel.cpp` (`computeEffectiveFrictionCoeff`) | **Exact 1:1** | Unit Test 3 (`Tire Friction-Circle & Load Sensitivity`) |
| 9 | Longitudinal Slip Ratio ($\kappa$) & Lateral Slip Angle ($\alpha$) from wheel contact velocity | `TireModel.ts` | `TireModel.cpp` | **Exact 1:1** | Unit Test 4 (`Wheel Angular Speed & Slip`) |
| 10 | Pacejka-Style Magic Formula Tire Curves & Combined-Slip Friction Circle ($\sqrt{F_x^2 + F_y^2} \le \mu_{\text{eff}} F_z$) | `TireModel.ts` | `TireModel.cpp` (`computeTireForcesFromSlip`) | **Exact 1:1** | Unit Test 3 & Validation Tests B, C, D, E |
| 11 | 8-Substep Wheel Rotational Inertia Integration ($\alpha_w = (T_{\text{drive}} - F_x r_w - T_{\text{brake}}\text{sgn}(\omega))/I_w$) | `VehicleDynamics.ts` | `VehicleDynamics.cpp` | **Exact 1:1** | Validation Tests B, C, E |
| 12 | Speed-Sensitive Steering Limit & Ackermann Inner/Outer Front Wheel Geometry | `SteeringSystem.ts` | `SteeringSystem.cpp` | **Exact 1:1** | Unit Test 7 (`Ackermann & Speed-Sensitive Steering`) |
| 13 | 9-Point Engine Torque Curve, Rev Limiter (`7000 RPM`), Closed-Throttle Engine Braking, Launch Clutch | `Drivetrain.ts` | `Drivetrain.cpp` | **Exact 1:1** | Unit Test 5 (`Drivetrain & Rev Limiter`) |
| 14 | 6-Speed + Reverse Gearbox (`AUTO` with hysteresis & `MANUAL` sequential shift with `0.14 s` cut) | `Drivetrain.ts` | `Drivetrain.cpp` | **Exact 1:1** | Unit Test 5 & Interactive HUD check |
| 15 | Selectable `RWD`, `FWD`, `AWD (40/60)` Layout & Viscous Limited-Slip Differential (LSD) | `Drivetrain.ts` | `Drivetrain.cpp` | **Exact 1:1** | Unit Test 5 & Live Tuning Lab |
| 16 | Front-Biased (`64% F / 36% R`) Service Brakes & Rear Mechanical Handbrake (`2600 N·m`) | `BrakeSystem.ts` | `BrakeSystem.cpp` | **Exact 1:1** | Unit Test 8 & Validation Test C |
| 17 | Quadratic Aerodynamic Drag, Speed-Squared Downforce, and Rolling Resistance | `Aerodynamics.ts` | `Aerodynamics.cpp` | **Exact 1:1** | Unit Test 8 & Validation Test J |
| 18 | 6 Road Surface Types (`dry_asphalt`, `wet_asphalt`, `gravel`, `ice`, `grass`, `rumble_strip`) | `SurfaceSystem.ts` | `SurfaceSystem.cpp` | **Exact 1:1** | Unit Test 6 & Validation Test E |
| 19 | Proving Ground Track (Main Straight, Wet Skidpad, Gravel Lane, Ice Strip, Bumps, Ramps, Slalom, Walls) | `PhysicsWorld.ts` | `PhysicsWorld.cpp` | **Exact 1:1** | Validation Tests F, H & Zone Presets `1`–`7` |

---

## 2. Rendering, Camera, Input & UI Parity

| # | Feature / Subsystem | Web Source (`src/rendering/`, `src/ui/`) | Native Implementation (`src/renderer/`, `src/input/`, `src/ui/`) | Parity Status | Verification Method |
| :-: | :--- | :--- | :--- | :---: | :--- |
| 20 | Multi-part Sports Coupe Mesh (body, hood, cabin, tinted glass, splitter, skirts, diffuser, spoiler, lights, interior steering wheel) | `VehicleRenderer.ts` | `VehicleRendererGL.cpp` | **Exact 1:1** | Visual & OpenGL render pass check |
| 21 | 4 Independent Wheel Groups (tire, 5-spoke rim, high-vis spoke stripe, rotor), non-spinning steered brake calipers, and scaling coilover struts | `VehicleRenderer.ts` | `VehicleRendererGL.cpp` | **Exact 1:1** | Visual & transform sync verification |
| 22 | Dynamic Brake Light & Reverse Light Emissive Response | `VehicleRenderer.ts` | `VehicleRendererGL.cpp` + `phong_shadow.frag` | **Exact 1:1** | Driven by `brake > 0.05` and `gearIndex == 0` |
| 23 | Procedural 256×256 Surface Textures (grid, noise, rumble stripes) & Reference Track Markers | `TrackRenderer.ts` | `Texture2D.cpp`, `TrackRendererGL.cpp` | **Exact 1:1** | OpenGL texture generation & UV repeat |
| 24 | Directional Sun Shadow Mapping (`2048x2048` High, `1024x1024` Medium, Off Low) following vehicle | `TrackRenderer.ts` | `ShadowMapFBO.cpp`, `phong_shadow.frag` | **Exact 1:1** | Quality selector & shadow FBO pass |
| 25 | 3D Force Vector & Friction Circle Debug Overlay (`G` key) | `DebugOverlay3D.ts` | `DebugOverlayGL.cpp` (`debug_line.vert/.frag`) | **Exact 1:1** | Renders CoM, basis, $\mathbf{v}$, $\mathbf{a}$, rays, $F_z$, $\mathbf{F}_{xy}$, friction rings, contacts |
| 26 | 4 Camera Modes (`Chase Follow`, `Orbit Inspection`, `Wheel & Suspension Rig`, `Driver Hood`) | `CameraController.ts` | `CameraControllerGL.cpp` | **Exact 1:1** | `C` key & mouse drag/wheel zoom in Orbit/Follow |
| 27 | Continuous Keyboard Pedal Ramping, Shortcuts (`W,A,S,D,Space,R,C,P,T,G,M,E,Q,1-7,Esc`) & Gamepad Support | `InputController.ts` | `InputControllerGLFW.cpp` | **Exact 1:1** | GLFW key & gamepad polling |
| 28 | Top Control Bar, 7 Zone Teleport Buttons, Quality Selector & `Verify Physics` Self-Check | `DashboardUI.ts` | `ImGuiDashboard.cpp` | **Exact 1:1** | Dear ImGui top toolbar & self-test runner |
| 29 | Bottom Instrument Cluster (Speed, Gear/Mode, RPM bar, Pedals, Steer, 2D G-G dot, Pitch/Roll, Surface badge) | `DashboardUI.ts` | `ImGuiDashboard.cpp` | **Exact 1:1** | Live `VehicleTelemetrySnapshot` binding |
| 30 | 4-Wheel Telemetry Table & 4-Band Real-Time Oscilloscope (`Speed`, `Accel`, `Pitch/Roll`, `Suspension`) | `TelemetryPanel.ts` | `ImGuiDashboard.cpp` (`ImDrawList` oscilloscope) | **Exact 1:1** | Live 180-sample circular history plot |
| 31 | Live Vehicle Tuning Lab (Drivetrain selector + 12 sliders + `Defaults` reset) | `TuningPanel.ts` | `ImGuiDashboard.cpp` | **Exact 1:1** | Updates `VehicleConfig` & `VehicleBody::syncMassProperties` |
