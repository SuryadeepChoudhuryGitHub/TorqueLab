# Blender Vehicle Modeling & glTF 2.0 Export Guidelines for Torque Lab

> **Torque Lab Native — Production Asset Specification & Compatibility Standard**
> This document provides 3D vehicle artists with the exact modeling, hierarchy, material, and export standards required for clean, 100% bug-free rendering and automatic physics derivation in Torque Lab Native.

---

## 1. Object Organization & Hierarchy

Torque Lab uses an intelligent semantic role mapper and automatic collision derivation pipeline. To ensure all parts (body shell, wheels, calipers, steering wheel) are identified correctly:

| Vehicle Role | Recommended Node Name | Purpose & Articulation |
| :--- | :--- | :--- |
| **Chassis (Body Shell)** | `Chassis`, `Body`, `Cabin`, `Hood`, `Door_L`, `Door_R` | Rigid chassis structure that pitches, rolls, and heaves with the vehicle body. |
| **Front-Left Wheel** | `Wheel_FL`, `Rim_FL`, `Tyre_FL` | Spins with drive/brake torque and steers with Ackermann angle. |
| **Front-Right Wheel** | `Wheel_FR`, `Rim_FR`, `Tyre_FR` | Spins with drive/brake torque and steers with Ackermann angle. |
| **Rear-Left Wheel** | `Wheel_RL`, `Rim_RL`, `Tyre_RL` | Driven/braked wheel assembly at rear axle. |
| **Rear-Right Wheel** | `Wheel_RR`, `Rim_RR`, `Tyre_RR` | Driven/braked wheel assembly at rear axle. |
| **Brake Calipers** | `Caliper_FL`, `Caliper_FR`, `Caliper_RL`, `Caliper_RR` | Steers with front knuckles but does NOT spin with wheels. |
| **Interior Steering Wheel** | `SteeringWheel`, `Steering_Wheel` | Rotates around local steering column axis proportional to input. |
| **Studio Exclusions** | `Ground`, `Plane_Studio`, `Camera`, `Light` | Excluded automatically or via manifest `ignore_nodes`. |

> [!TIP]
> If modeling multiple subparts per wheel (e.g., Rim, Tire, Center Cap, Lug Nuts, Brake Disc), either parent them under a parent wheel empty/node, or name them with consistent quadrant suffixes (e.g., `Rim_FL`, `Tire_FL`, `Disc_FL`).

---

## 2. Applying Transforms (Eliminate Negative Scale & Inverted Winding)

A common cause of inverted face winding and disappearing backfaces is unapplied negative scaling (e.g. duplicating a left-side mirror or step bar to the right side using `Scale X = -1.0`).

### The Golden Rule:
Before exporting to glTF 2.0:
1. In the 3D Viewport, select all objects (`A`).
2. Press **`Ctrl + A`** and choose **`Apply All Transforms`** (Location, Rotation, Scale).
3. Verify in the Item Properties panel (`N`) that:
   - Scale is strictly **`(1.0, 1.0, 1.0)`** for every object.
   - Rotation is strictly **`(0°, 0°, 0°)`** for every object.

Torque Lab Native automatically detects negative determinants at load time and reverses triangle index winding ($i_1 \leftrightarrow i_2$), but applying transforms in Blender guarantees zero geometry ambiguity.

---

## 3. Normal Orientation & Winding

Torque Lab renders opaque vehicle components with hardware backface culling (`GL_BACK`, CCW winding) enabled for maximum framerate performance.

### Verification Steps:
1. In the Viewport Overlays dropdown (top right of the 3D Viewport), enable **`Face Orientation`**.
2. Inspect the vehicle:
   - **Blue** surfaces face outward (exterior).
   - **Red** surfaces face inward (interior / backface).
3. If any exterior body panel, door, or rim appears **Red**:
   - Tab into **Edit Mode** (`Tab`), select all faces (`A`).
   - Press **`Shift + N`** (**Recalculate Outside**).
   - Alternatively, use **Mesh > Normals > Flip** on inverted face selections.

---

## 4. Material Setup & PBR Shading Standards

Torque Lab uses an industry-standard metallic/roughness physically based rendering (PBR) pipeline adhering to the glTF 2.0 specification:

### Material Channel Mapping:
- **Base Color (Albedo):** sRGB linear color or texture map (Unit 0).
- **Metallic:** Scalar $0.0$ (dielectrics like paint, rubber, leather) to $1.0$ (bare chrome, aluminum, steel).
- **Roughness:** Scalar $0.04$ (mirror-smooth gloss) to $1.0$ (rough rubber, matte plastic).
- **Normal Map:** Tangent-space normal map (RGB = XYZ normal perturbation).
- **Ambient Occlusion:** Red channel of the occlusion map.
- **Emissive:** Color and intensity for headlights, taillights, and dashboard displays.

### Material Naming Conventions:
Torque Lab automatically detects semantic vehicle roles from material names if manifest overrides are not provided:

| Material Type | Keyword in Material Name | Engine Handling |
| :--- | :--- | :--- |
| **Car Body Paint** | `paint`, `body`, `car_paint` | Opaque PBR with clearcoat specular reflections. |
| **Tire Rubber** | `tire`, `tyre`, `rubber` | Opaque matte rubber ($R=0.82$, $M=0.0$). Undergoes physical pneumatic squash. |
| **Rim / Alloy** | `rim`, `alloy`, `wheel` | High metallic finish ($M=0.85$, $R=0.20$). Protected from deformation. |
| **Cabin Glass** | `glass`, `windshield`, `window` | Transparent pass with depth sorting ($O=0.30$, Blend mode). |
| **Brake Lights** | `brakelight`, `tailight`, `rearlight` | Reactive emissive material (illuminates bright red when braking). |
| **Reverse Lights** | `reverse_light` | Reactive emissive material (illuminates bright white in reverse). |
| **Chrome / Mirror** | `chrome`, `mirror` | Ultra-reflective dielectric/metal ($M=0.99$, $R=0.02$). |

---

## 5. Double-Sided Mesh Handling & Thin Panels

Blender meshes that represent zero-thickness sheet metal (e.g., hood louvers, front grille slats, underbody skid plates, fender liners):

- In Blender Material Settings (under Settings > Surface), **Double Sided** is enabled by default.
- In Torque Lab, any material marked `doubleSided` in the glTF will automatically toggle backface culling off (`glDisable(GL_CULL_FACE)`) during its draw call and invert the surface shading normal in the fragment shader for back-facing fragments.
- **Best Practice for Structural Parts:** For primary exterior body panels (doors, fenders, hood), give the mesh actual thickness using the **Solidify Modifier** ($5\text{ mm} - 10\text{ mm}$ thickness) before export. This prevents self-shadowing artifacts and gives realistic door jambs and hood seams.

---

## 6. Transparency vs. Opaque Bodywork (Preventing "Cloudy" Body Shells)

A frequent issue in imported models is body panels appearing transparent or ghostly.

### Why It Happens:
In Blender Eevee, materials often inherit **Blend Mode: Alpha Blend** or **Alpha Hashed** by accident, which causes the Blender glTF exporter to set `alphaMode: "BLEND"` on opaque car paint or metal!

### How to Configure in Blender:
1. Select the material in Blender's Material Properties tab.
2. Scroll down to the **Settings** panel:
   - Set **Blend Mode** to **`Opaque`**.
   - Set **Shadow Mode** to **`Opaque`**.
3. **Only** set **Blend Mode** to **`Alpha Blend`** for genuine clear glass (`windshield`, `side_glass`, `headlight_lens`).

> [!NOTE]
> Torque Lab Native includes an automated **Bodywork Transparency Leakage Guard** in `ModelLoaderGLB.cpp` that automatically forces body paint, metals, and rubbers to Opaque even if exported with accidental blend flags.

---

## 7. Wheel Pivot Centering

For realistic wheel spin and suspension articulation:
1. Select each wheel assembly object in Blender.
2. Set the object origin exactly to the wheel rotation center:
   - In Edit Mode, select the circular rim bead or hub vertices.
   - Press **`Shift + S`** -> **`Cursor to Selected`**.
   - Tab into Object Mode, right-click -> **`Set Origin`** -> **`Origin to 3D Cursor`**.
3. Torque Lab automatically calculates wheel bounding extents and pivot centers from the geometry, but having the origin at the hub center ensures exact sub-part alignment.

---

## 8. Blender glTF 2.0 Export Settings

When exporting the model from Blender (`File > Export > glTF 2.0 (.glb/.gltf)`):

| Setting Category | Recommended Option | Explanation |
| :--- | :--- | :--- |
| **Format** | **`glTF Binary (.glb)`** | Self-contained single file containing all geometry, hierarchies, and embedded textures. |
| **Include** | **`Visible Objects`** or **`Selected Objects`** | Avoids exporting hidden helper objects or cameras. |
| **Transform** | **`+Y Up`** (Checked) | Converts Blender's +Z up coordinate system to standard glTF +Y up. |
| **Geometry** | **`Apply Modifiers`** (Checked) | Applies Subsurf, Bevel, Mirror, and Solidify modifiers. |
| **Geometry** | **`Normals`** (Checked) | Exports custom split normals and smoothing groups. |
| **Geometry** | **`Tangents`** (Checked) | Exports precomputed MikkTSpace tangent vectors for normal mapping. |
| **Materials** | **`Export Materials`** = **`Export`** | Exports PBR metallic/roughness material definitions. |
| **Images** | **`Automatic`** | Embeds PNG/JPEG textures into binary GLB buffer chunks. |

---

## 9. Manifest Configuration Reference

If a model requires custom scaling, rotation, or semantic node mappings, place a manifest JSON file with the same base name next to the model (e.g., `model/MyCar_manifest.json`):

```json
{
  "model_name": "My Custom Vehicle",
  "source": "Blender glTF 2.0 Export",
  "scale": 1.0,
  "rotation_y_deg": 0.0,
  "ignore_nodes": [
    "Ground",
    "Camera",
    "Light"
  ],
  "roles": {
    "Wheel_FL_Node": "Wheel_FL",
    "Wheel_FR_Node": "Wheel_FR",
    "Wheel_RL_Node": "Wheel_RL",
    "Wheel_RR_Node": "Wheel_RR",
    "Caliper_FL_Node": "BrakeCaliper_FL",
    "Caliper_FR_Node": "BrakeCaliper_FR",
    "Caliper_RL_Node": "BrakeCaliper_RL",
    "Caliper_RR_Node": "BrakeCaliper_RR",
    "Steering_Wheel_Node": "SteeringWheel"
  },
  "material_overrides": {
    "car_paint": { "roughness": 0.22, "metalness": 0.55, "base_color_hex": "#102a45" },
    "glass": { "roughness": 0.05, "metalness": 0.15, "opacity": 0.30, "transparent": true, "base_color_hex": "#0f172a" },
    "brakelight": { "roughness": 0.20, "metalness": 0.10, "is_brake_light": true, "base_color_hex": "#780808", "emissive_color_hex": "#ff1e1e" }
  }
}
```

---

## 10. Automated Validation

Every vehicle model loaded in Torque Lab Native is automatically evaluated by the `VehicleAssetValidator`. You can run asset diagnostics at any time via CLI:

```powershell
.\build_gcc\TorqueLabNative.exe --smoke-test
.\build_gcc\TorqueLabNative.exe --diag
```

The validator verifies:
- $\ge 1,000$ vertices and $\ge 500$ triangles.
- 0 degenerate faces.
- 0 bodywork transparency leaks.
- 4/4 wheel quadrants identified with plausible wheelbase ($2.0\text{m} - 4.5\text{m}$) and track ($1.2\text{m} - 2.5\text{m}$).
- Proper double-sided culling adaptation.
