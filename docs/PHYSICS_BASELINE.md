# Torque Lab — Physics Baseline & Mathematical Specification (`PHYSICS_BASELINE.md`)

This document records the exact mathematical equations, coordinate conventions, SI parameter values, and reproducible validation test criteria extracted from the reference web implementation (`src/physics/*.ts`) to serve as the mathematical contract for the native C++20 migration.

---

## 1. Coordinate System & Units

- **Units:** Strict SI units throughout all calculations:
  - Length: meters ($\text{m}$)
  - Mass: kilograms ($\text{kg}$)
  - Time: seconds ($\text{s}$)
  - Force: Newtons ($\text{N} = \text{kg}\cdot\text{m}/\text{s}^2$)
  - Torque: Newton-meters ($\text{N}\cdot\text{m}$)
  - Angles: radians ($\text{rad}$), angular velocity in $\text{rad}/\text{s}$, engine speed in $\text{RPM}$
  - Gravitational acceleration: $g = 9.81\text{ m/s}^2$ along $-Y$, i.e. $\mathbf{g} = (0, -9.81, 0)\text{ m/s}^2$.
- **Right-Handed Coordinate Frame (World & Vehicle Local):**
  - $+X$: Vehicle **Left** (`-X` is Right)
  - $+Y$: Vehicle **Up** (`-Y` is Down)
  - $+Z$: Vehicle **Forward** (`-Z` is Rear)
  - Verification of right-handedness: $\hat{\mathbf{e}}_x \times \hat{\mathbf{e}}_y = \text{Left} \times \text{Up} = \text{Forward} = +\hat{\mathbf{e}}_z$.
- **Euler Attitude Sign Conventions (`getVehicleEulerAngles`):**
  - **Pitch ($\theta$):** $\theta = \arcsin(\text{clamp}(\text{forward}_y, -1, 1))$.
    - Nose **Down** (braking dive): $\theta < 0$.
    - Nose **Up** (acceleration squat): $\theta > 0$.
  - **Roll ($\phi$):** $\phi = \text{atan2}(-\text{left}_y, \text{up}_y)$.
    - Leaning **Right** (left-hand turn outward roll): $\phi > 0$.
    - Leaning **Left** (right-hand turn outward roll): $\phi < 0$.
  - **Yaw ($\psi$):** $\psi = \text{atan2}(\text{forward}_x, \text{forward}_z)$.

---

## 2. Governing Physical Equations

### 2.1 Center of Mass & Wheel Mount Geometry
Given wheelbase $L$ (`2.70 m`) and track width $T$ (`1.60 m`):
- Local wheel upper suspension mounts:
  - Front-Left (`0, FL`): $(+T/2, h_{\text{mount}}, +L/2)$
  - Front-Right (`1, FR`): $(-T/2, h_{\text{mount}}, +L/2)$
  - Rear-Left (`2, RL`): $(+T/2, h_{\text{mount}}, -L/2)$
  - Rear-Right (`3, RR`): $(-T/2, h_{\text{mount}}, -L/2)$
- Local Center of Mass offset from geometric chassis origin:
  $$\mathbf{r}_{\text{CoM}} = \bigl(0,\; h_{\text{CoM}},\; (w_{\text{front}} - 0.5)\cdot L\bigr)$$

### 2.2 Raycast Suspension, Bump Stops & Anti-Roll Bars
For each wheel $i \in \{0, 1, 2, 3\}$, a ray of maximum length $L_{\text{ray}} = l_{\max} + r_w$ is cast from the world mount point $\mathbf{p}_{\text{mount},i}$ along the chassis downward direction $-\hat{\mathbf{u}}_{\text{up}}$.
- If a hit occurs at distance $d_{\text{hit}} \le l_{\text{rest}} + r_w$:
  - Raw suspension length: $l_{\text{raw}} = d_{\text{hit}} - r_w$
  - Clamped suspension length: $l_i = \text{clamp}(l_{\text{raw}}, l_{\min}, l_{\max})$
  - Displacement from rest: $x_i = l_i - l_{\text{rest}}$ (negative when compressed)
  - Compression distance: $c_i = \max(0, l_{\text{rest}} - l_i)$
  - Suspension compression rate along strut: $v_{s,i} = \mathbf{v}_{\text{contact},i} \cdot (-\hat{\mathbf{u}}_{\text{up}})$, so $x_i$ rate of change is $\dot{x}_i = -v_{s,i}$.
- **Spring + Asymmetric Damper Force:**
  $$F_{\text{spring},i} = -k_i x_i, \qquad F_{\text{damper},i} = -c_{\text{damp},i} \dot{x}_i, \qquad c_{\text{damp},i} = \begin{cases} c_{\text{bound},i} & \dot{x}_i < 0 \\ c_{\text{rebound},i} & \dot{x}_i \ge 0 \end{cases}$$
- **Progressive Bump Stop Force (when $l_{\text{raw}} < l_{\min} + 0.03\text{ m}$):**
  $$\delta_b = (l_{\min} + 0.03) - l_{\text{raw}}, \qquad F_{\text{bump},i} = k_{\text{bump}}\delta_b + 3 k_{\text{bump}}\delta_b^2$$
- **Axle Anti-Roll Bar Force:**
  $$F_{\text{arb},L} = k_{\text{arb}}(c_L - c_R), \qquad F_{\text{arb},R} = -F_{\text{arb},L}$$
- **Normal Tire Load $F_{z,i}$ (Projected onto Contact Normal $\hat{\mathbf{n}}_i$):**
  $$\cos\gamma_i = \max\bigl(0.25,\; \hat{\mathbf{n}}_i \cdot \hat{\mathbf{u}}_{\text{up}}\bigr), \qquad F_{z,i} = \max\bigl(0,\; (F_{\text{spring},i} + F_{\text{damper},i} + F_{\text{bump},i} + F_{\text{arb},i})\cos\gamma_i\bigr)$$

### 2.3 Tire Slip, Pacejka Curve, Load Sensitivity & Friction Circle
Let $v_x = \mathbf{v}_{\text{patch}} \cdot \hat{\mathbf{t}}_{\text{fwd}}$ and $v_y = \mathbf{v}_{\text{patch}} \cdot \hat{\mathbf{t}}_{\text{left}}$ be the contact-patch velocities in the steered wheel plane.
- **Longitudinal Slip Ratio ($\kappa$):**
  $$v_{\text{ref}} = \max(|v_x|, 1.25\text{ m/s}), \qquad \kappa = \begin{cases} 0 & |v_x| < 0.08 \text{ and } |\omega r_w| < 0.08 \\ \text{clamp}\!\left(\frac{\omega r_w - v_x}{v_{\text{ref}}}, -3.0, 3.0\right) & \text{otherwise} \end{cases}$$
- **Lateral Slip Angle ($\alpha$):**
  $$\alpha = \begin{cases} 0 & |v_x| < 0.15 \text{ and } |v_y| < 0.08 \\ -\text{atan2}\bigl(v_y,\; \max(|v_x|, 0.8\text{ m/s})\bigr) \cdot \min\!\left(1, \frac{\sqrt{v_x^2 + v_y^2}}{0.6}\right) & \text{otherwise} \end{cases}$$
- **Load-Sensitive Friction Coefficient:**
  $$\mu_{\text{eff}}(F_z) = \mu_0 \cdot \mu_{\text{surface}} \cdot \text{clamp}\!\left(1 - k_{\text{load}}\frac{F_z - F_{z,\text{nom}}}{F_{z,\text{nom}}},\; 0.55,\; 1.15\right)$$
  $$F_{\max} = \mu_{\text{eff}}(F_z) \cdot F_z$$
- **Pacejka-Style Single-Axis Curve:**
  $$\Phi(s, B, C, D, E) = D \sin\!\Bigl(C \arctan\bigl(Bs - E(Bs - \arctan(Bs))\bigr)\Bigr)$$
  with $C_x = 1.65, E_x = 0.35, B_x = k_{\text{long}} k_{\text{surf}}$ and $C_y = 1.45, E_y = 0.20, B_y = k_{\text{lat}} k_{\text{surf}}$.
- **Combined-Slip Friction Circle Saturation:**
  Given normalized slips $s_x = \kappa / 0.11$, $s_y = \alpha / 0.14$, and $s_{\text{comb}} = \sqrt{s_x^2 + s_y^2}$:
  - Directional allocation weights blend smoothly from independent linear slope at $s_{\text{comb}} < 0.7$ to pure direction cosine $(|s_x|/s_{\text{comb}}, |s_y|/s_{\text{comb}})$ at saturation ($s_{\text{comb}} \ge 1.3$).
  - Finally, hard-clamp magnitude: if $\sqrt{F_x^2 + F_y^2} > F_{\max}$, scale both $F_x, F_y$ by $F_{\max} / \sqrt{F_x^2 + F_y^2}$.

### 2.4 Sub-Stepped Wheel Rotational Dynamics
Over $N_{\text{sub}} = 8$ substeps per physics step ($\delta t = \Delta t / 8$):
1. Integrate drive and longitudinal road reaction torque:
   $$\omega' = \omega + \frac{T_{\text{drive}} - F_x(\omega) r_w}{I_w} \delta t$$
2. Apply passive resistive torque $T_{\text{passive}} = T_{\text{brake}} + T_{\text{roll}}$ opposing $\omega'$ without crossing zero unless $|T_{\text{drive}} - F_x r_w| > T_{\text{passive}}$.

### 2.5 Steering, Drivetrain, Brakes & Aerodynamics
- **Speed-Sensitive Ackermann Steering:**
  $$\delta_{\max}(v) = \frac{\delta_0}{1 + k_v |v_{\text{fwd}}|}, \qquad \delta_{\text{in}} = \arctan\!\left(\frac{L}{R - T/2}\right), \qquad \delta_{\text{out}} = \arctan\!\left(\frac{L}{R + T/2}\right)$$
  blended by `ackermannFactor` $= 0.85$.
- **Drivetrain & Viscous LSD:**
  - Engine torque $T_{\text{eng}} = \text{Curve}(\text{RPM}) \cdot m_{\text{torque}} \cdot \text{throttle} - T_{\text{eb}}(1 - \text{throttle})$, cut to $-T_{\text{eb}}$ when $\text{RPM} \ge \text{RPM}_{\max}$ or during a `0.14 s` gearshift.
  - Total gear ratio $G_{\text{total}} = g_{\text{gear}} \cdot g_{\text{final}}$.
  - Viscous LSD locking torque on driven axle: $\Delta T_{\text{lock}} = \text{clamp}(k_{\text{lsd}}(\omega_L - \omega_R), -0.45 |T_{\text{axle}}|, +0.45 |T_{\text{axle}}|)$.
- **Aerodynamics:**
  $$\mathbf{F}_{\text{drag}} = -\frac{1}{2}\rho C_d A |\mathbf{v}|\mathbf{v}, \qquad \mathbf{F}_{\text{down}} = -\frac{1}{2}\rho C_L A v_{\text{fwd}}^2 \hat{\mathbf{u}}_{\text{up}}$$

---

## 3. Baseline Parameter Values (`createDefaultVehicleConfig`)

| Parameter | Symbol / Field | Baseline Value | SI Unit |
| :--- | :--- | :---: | :---: |
| Vehicle Mass | `mass` | `1500` | $\text{kg}$ |
| CoM Vertical Offset | `comHeightOffset` | `0.0` | $\text{m}$ |
| Front Weight Distribution | `frontWeightDistribution` | `0.52` | ratio |
| Principal Inertia Tensor | `inertiaTensor` | `(2450, 2650, 680)` | $\text{kg}\cdot\text{m}^2$ |
| Wheelbase / Track Width | `wheelbase` / `trackWidth` | `2.70` / `1.60` | $\text{m}$ |
| Wheel Radius / Width / Inertia | `wheelRadius` / `wheelWidth` / `wheelInertia` | `0.32` / `0.235` / `1.15` | $\text{m}, \text{m}, \text{kg}\cdot\text{m}^2$ |
| Suspension Rest / Min / Max | `suspensionRestLength` / `Min` / `Max` | `0.38` / `0.15` / `0.48` | $\text{m}$ |
| Front / Rear Spring Stiffness | `frontSpringStiffness` / `rearSpringStiffness` | `35000` / `32500` | $\text{N/m}$ |
| Front Compression / Rebound Damping | `frontDampingCompression` / `Rebound` | `3600` / `4800` | $\text{N}\cdot\text{s/m}$ |
| Rear Compression / Rebound Damping | `rearDampingCompression` / `Rebound` | `3400` / `4500` | $\text{N}\cdot\text{s/m}$ |
| Bump Stop Stiffness | `bumpStopStiffness` | `180000` | $\text{N/m}$ |
| Front / Rear Anti-Roll Stiffness | `frontAntiRollStiffness` / `rearAntiRollStiffness` | `16000` / `12000` | $\text{N/m}$ |
| Tire Friction / Stiffness / Load Sens. | `tireFrictionCoeff` / `Long` / `Lat` / `LoadSens` | `1.10` / `13.0` / `11.5` / `0.12` | — |
| Nominal Tire Load / Force Height Offset | `nominalTireLoad` / `tireForceApplicationHeightOffset` | `3700` / `0.06` | $\text{N}, \text{m}$ |
| Max Steer / Rate / Return / Speed Sens. | `maxSteeringAngle` / `Speed` / `Return` / `Sens` | `0.60` / `2.6` / `3.4` / `0.032` | $\text{rad}, \text{rad/s}, \text{s/m}$ |
| Drivetrain Layout & Gear Ratios | `RWD`, `[-3.20, 0.0, 3.45, 2.15, 1.52, 1.18, 0.96, 0.79]` | Final Drive `3.73`, Eff `0.88` | — |
| Service Brake / Bias / Handbrake | `maxBrakeTorque` / `brakeFrontBias` / `handbrakeTorque` | `3600` / `0.64` / `2600` | $\text{N}\cdot\text{m}$, ratio |
| Air Density / Drag $C_d$ / Area $A$ / Downforce $C_L$ | `airDensity` / `dragCoefficient` / `frontalArea` / `downforceCoefficient` | `1.225` / `0.31` / `2.15` / `0.18` | $\text{kg/m}^3, -, \text{m}^2, -$ |

---

## 4. Reproducible Validation Scenarios (`Tests A – J`)

1. **Test A — Static Equilibrium:** Spawn at $y = 0.60\text{ m}$, step `360` frames (`3.0 s` at `120 Hz`). Verify $\sum F_{z,i} \approx mg = 14715\text{ N}$ within `1.5%`, all 4 wheels in contact with compression $\in [0.08, 0.16]\text{ m}$, and $|v_y| < 0.005\text{ m/s}$.
2. **Test B — Acceleration & Rear Squat:** Settle `120` steps, apply `throttle = 1.0` for `240` steps (`2.0 s`). Verify $v_{\text{fwd}} > 10.0\text{ m/s}$, $a_{\text{long}} > 2.5\text{ m/s}^2$, rear load increases by $> 300\text{ N}$, front load decreases by $> 300\text{ N}$, and pitch $\theta > +0.004\text{ rad}$ (nose up).
3. **Test C — Braking & Front Dive:** Accelerate to $22\text{ m/s}$, apply `brake = 0.85` for `60` steps (`0.5 s`). Verify progressive deceleration ($a_{\text{long}} < -5.0\text{ m/s}^2$), front load exceeds rear load by $> 1500\text{ N}$, and pitch $\theta < -0.008\text{ rad}$ (nose down).
4. **Test D — Cornering & Roll Reversal:** At $18\text{ m/s}$, steer left (`+0.55`) for `90` steps ($\phi > +0.01\text{ rad}$, right outside load $>$ left inside load $+ 1200\text{ N}$), then steer right (`-0.55`) for `140` steps ($\phi < -0.01\text{ rad}$, left outside load $>$ right inside load $+ 1200\text{ N}$).
5. **Test E — Surface Grip Comparison (`dry_asphalt` vs `ice`):** Verify launch slip ratio on ice $> 1.5\times$ dry, launch speed on dry $> 2.0\times$ ice, and braking distance from $15\text{ m/s}$ on ice $> 2.5\times$ dry.
6. **Test F — Speed Bump Damping:** Drive over a $0.05\text{ m}$ bump at $12\text{ m/s}$; verify peak compression $> c_{\text{eq}} + 0.035\text{ m}$ and subsequent oscillation damping.
7. **Test G — Parameter Sensitivity:** Verify $1.6\times$ stiffer springs reduce static compression below $0.72\times$ baseline, and raising CoM height from $-0.10\text{ m}$ to $+0.28\text{ m}$ increases cornering roll by $> 1.25\times$.
8. **Test H — Collision & Recovery Reset:** Crash into wall at $14\text{ m}$ from $15\text{ m/s}$; verify stop before $z = 13.0\text{ m}$, finite telemetry sanity, and clean reset to origin.
9. **Test I — Timestep Independence:** Run `2.0 s` curved acceleration at `30 FPS`, `60 FPS`, and `120 FPS` outer frame rates; verify position and speed match within $0.25\text{ m}$ and $0.20\text{ m/s}$.
10. **Test J — Coasting Energy Sanity:** Coast in Neutral from $25\text{ m/s}$ for `360` steps; verify strictly monotonic speed decrease due to drag and rolling resistance.
