/**
 * Torque Lab Native — Physically Based Deformable Pneumatic Tyre Simulation Implementation.
 */

#include "vehicle/PneumaticTire.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include "vehicle/VehicleConfig.h"

namespace torquelab {

PneumaticTireModel::PneumaticTireModel(const PneumaticTireProperties& props)
    : m_props(props)
{
    reset(m_props.nominalPressurePa, m_props.ambientTempC);
}

void PneumaticTireModel::reset(double initialPressurePa, double initialTempC) {
    m_state = PneumaticTireState{};
    m_state.nominalPressurePa = m_props.nominalPressurePa;
    m_state.pressurePa = clamp(initialPressurePa, m_props.minSafePressurePa * 0.2, m_props.maxSafePressurePa * 1.5);
    m_state.temperatureC = initialTempC;
    m_state.wearLevel = 0.0;
    m_state.damageLevel = 0.0;
    m_state.punctureRatePaPerSec = 0.0;
    m_state.isPunctured = false;
    m_state.isBlown = false;

    updateDynamicStiffness();

    m_state.effectiveRadius = m_props.outerRadius;
    m_state.contactPatchLength = 0.0;
    m_state.contactPatchWidth = m_props.sectionWidth;
    m_state.contactPatchArea = 0.0;

    buildSampleGrid(WheelCollisionFidelity::Balanced);
}

void PneumaticTireModel::updateDynamicStiffness() {
    m_state.radialStiffness = computeRadialStiffness(m_state.pressurePa);
    m_state.lateralStiffness = computeLateralStiffness(m_state.pressurePa);
    m_state.longitudinalStiffness = computeLongitudinalStiffness(m_state.pressurePa);
}

double PneumaticTireModel::computeRadialStiffness(double p) const {
    // k_r(p) = k_r0 + c_pr * (p - p0)
    // Physical guard: minimum residual stiffness even if completely flat (bead/sidewall rubber only)
    const double dp = p - m_props.nominalPressurePa;
    const double k = m_props.radialStiffnessBase + m_props.radialStiffnessPressureCoeff * dp;
    return std::max(45000.0, k);
}

double PneumaticTireModel::computeLateralStiffness(double p) const {
    const double dp = p - m_props.nominalPressurePa;
    const double k = m_props.lateralStiffnessBase + m_props.lateralStiffnessPressureCoeff * dp;
    return std::max(35000.0, k);
}

double PneumaticTireModel::computeLongitudinalStiffness(double p) const {
    const double dp = p - m_props.nominalPressurePa;
    const double k = m_props.longitudinalStiffnessBase + m_props.longitudinalStiffnessPressureCoeff * dp;
    return std::max(40000.0, k);
}

void PneumaticTireModel::setTargetPressure(double pressurePa) {
    m_state.pressurePa = clamp(pressurePa, 20000.0, m_props.maxSafePressurePa * 1.5);
    updateDynamicStiffness();
}

void PneumaticTireModel::setPunctured(bool punctured, double leakRatePaPerSec) {
    m_state.isPunctured = punctured;
    m_state.punctureRatePaPerSec = punctured ? std::max(500.0, leakRatePaPerSec) : 0.0;
}

void PneumaticTireModel::updatePneumaticState(double dt) {
    // 1. Puncture leakage
    if (m_state.isPunctured && m_state.punctureRatePaPerSec > 0.0) {
        m_state.pressurePa = std::max(15000.0, m_state.pressurePa - m_state.punctureRatePaPerSec * dt);
        if (m_state.pressurePa < m_props.minSafePressurePa * 0.4) {
            m_state.damageLevel = std::min(1.0, m_state.damageLevel + 0.02 * dt);
        }
    }

    // 2. Ideal gas thermal pressure adjustment: p(T) = p_cold * (T + 273.15) / (T_cold + 273.15)
    // Small realistic expansion/contraction with temperature (~1.5 PSI per 10 deg C)
    const double tempKelvin = m_state.temperatureC + 273.15;
    const double refKelvin = m_props.ambientTempC + 273.15;
    const double thermalFactor = clamp(tempKelvin / refKelvin, 0.85, 1.30);
    const double effectiveP = m_state.pressurePa * (0.90 + 0.10 * thermalFactor);

    // 3. Update dynamic structural carcass stiffnesses
    m_state.radialStiffness = computeRadialStiffness(effectiveP);
    m_state.lateralStiffness = computeLateralStiffness(effectiveP);
    m_state.longitudinalStiffness = computeLongitudinalStiffness(effectiveP);
}

void PneumaticTireModel::computeContactPatchDimensions(double normalLoad, double& outLength, double& outWidth) const {
    if (normalLoad <= 1.0) {
        outLength = 0.0;
        outWidth = m_props.sectionWidth;
        return;
    }

    // Analytical pneumatic contact footprint model:
    // Footprint length L_p = 2 * sqrt(R_0^2 - (R_0 - delta_r)^2) ~= 2 * sqrt(2 * R_0 * delta_r)
    // where delta_r ~= F_z / k_r(p)
    const double delta_r = clamp(normalLoad / m_state.radialStiffness, 0.0, m_props.sidewallHeight * 0.85);
    const double r0 = m_props.outerRadius;
    const double halfLen = std::sqrt(std::max(0.0, 2.0 * r0 * delta_r - delta_r * delta_r));
    outLength = clamp(2.0 * halfLen, 0.04, 0.38);

    // Contact patch width expands slightly under compression due to rubber Poisson expansion
    const double widthExpansion = m_props.poissonRatio * (delta_r / m_props.sidewallHeight) * 0.04;
    outWidth = clamp(m_props.sectionWidth + widthExpansion, m_props.sectionWidth, m_props.sectionWidth * 1.25);
}

double PneumaticTireModel::computeThermalGripFactor(double tempC) const {
    // Parabolic bell curve peaked at optimalTempC:
    // mu(T) = mu_opt * (1.0 - 0.28 * ((T - T_opt) / halfWidth)^2)
    const double diff = (tempC - m_props.optimalTempC) / m_props.tempGripHalfWidthC;
    const double factor = 1.0 - 0.28 * diff * diff;
    return clamp(factor, 0.65, 1.05);
}

void PneumaticTireModel::updateThermodynamics(double dt, double vehicleSpeedMps) {
    // 1. Heat Generation Q_in:
    // Q_slip = frictional work from sliding contact
    // Q_flex = cyclic hysteresis loss from rolling rubber deflection
    const double qSlip = m_state.totalFrictionalPowerWatts;
    const double qFlex = m_state.totalFlexurePowerWatts;
    const double totalQin = qSlip + qFlex;

    // 2. Heat Dissipation Q_out:
    // Convective air cooling proportional to vehicle forward speed + conduction to road
    const double hAir = m_props.heatTransferAirBase + m_props.heatTransferAirSpeed * std::abs(vehicleSpeedMps);
    const double deltaT_Air = m_state.temperatureC - m_props.ambientTempC;
    const double qCoolAir = hAir * deltaT_Air;

    // Conduction to ground pavement across active contact patch
    const double qCoolRoad = (m_state.activeContactSamples > 0)
        ? m_props.heatTransferRoad * deltaT_Air
        : 0.0;
    const double totalQout = qCoolAir + qCoolRoad;

    // 3. Thermal Mass Integration: dT/dt = (Q_in - Q_out) / C_th
    const double deltaTemp = ((totalQin - totalQout) / m_props.thermalMass) * dt;
    m_state.temperatureC = clamp(m_state.temperatureC + deltaTemp, -20.0, 220.0);

    // 4. Update thermal grip scaling factor
    m_state.thermalGripMultiplier = computeThermalGripFactor(m_state.temperatureC);
}

void PneumaticTireModel::updateWear(double dt) {
    if (m_state.totalFrictionalPowerWatts > 10.0) {
        // Wear accumulates from sliding friction energy: dW = k_wear * P_fric * dt
        const double deltaWear = m_props.wearRatePerJoule * m_state.totalFrictionalPowerWatts * dt;
        m_state.wearLevel = clamp(m_state.wearLevel + deltaWear, 0.0, 1.0);
    }
}

void PneumaticTireModel::buildSampleGrid(WheelCollisionFidelity fidelity) {
    m_state.contactSamples.clear();

    int nU = 3; // Longitudinal slices
    int nV = 3; // Lateral slices

    if (fidelity == WheelCollisionFidelity::Performance) {
        nU = 3;
        nV = 1;
    } else if (fidelity == WheelCollisionFidelity::Detailed) {
        nU = 5;
        nV = 3;
    } else { // Balanced
        nU = 3;
        nV = 3;
    }

    m_state.totalContactSamples = nU * nV;
    m_state.contactSamples.resize(static_cast<size_t>(m_state.totalContactSamples));

    int idx = 0;
    for (int i = 0; i < nU; ++i) {
        // Longitudinal coordinate u in [-1..+1]
        const double u = (nU > 1) ? (-1.0 + 2.0 * static_cast<double>(i) / (nU - 1)) : 0.0;
        for (int j = 0; j < nV; ++j) {
            // Lateral coordinate v in [-1..+1]
            const double v = (nV > 1) ? (-1.0 + 2.0 * static_cast<double>(j) / (nV - 1)) : 0.0;

            TireContactPointSample& s = m_state.contactSamples[static_cast<size_t>(idx)];
            s.index = idx;
            s.normalizedU = u;
            s.normalizedV = v;
            idx++;
        }
    }
}

void PneumaticTireModel::evaluateDistributedContact(
    double dt,
    double normalLoadRequested,
    double rawSuspensionTravel,
    double wheelAngularVelocity,
    const Vec3& hubCenterWorld,
    const Quat& wheelWorldOrientation,
    const VehicleBasis& steeredBasis,
    const Vec3& chassisDown,
    const SurfaceProperties& surface,
    WheelCollisionFidelity fidelity,
    const std::vector<std::pair<double, double>>& contactHitPoints,
    double chassisVx,
    double chassisVy
) {
    // Ensure grid matches current fidelity mode
    if (m_state.contactSamples.empty() ||
        (fidelity == WheelCollisionFidelity::Performance && m_state.totalContactSamples != 3) ||
        (fidelity == WheelCollisionFidelity::Balanced && m_state.totalContactSamples != 9) ||
        (fidelity == WheelCollisionFidelity::Detailed && m_state.totalContactSamples != 15)) {
        buildSampleGrid(fidelity);
    }

    updatePneumaticState(dt);

    const double r0 = m_props.outerRadius;
    const double rRim = m_props.rimRadius;
    const double maxRadialDeflection = std::max(0.01, (r0 - rRim) * 0.85); // Rim clearance limit

    // 1. Calculate static equilibrium deformation target from requested normal load
    const double targetRadialDeflection = (normalLoadRequested > 0.0 && m_state.radialStiffness > 1e3)
        ? clamp(normalLoadRequested / m_state.radialStiffness, 0.0, maxRadialDeflection)
        : 0.0;

    // 2. Viscoelastic Kelvin-Voigt structural integration for dynamic radial squash delta_r
    // delta_r_dot = (target - current) / tau, with structural damping preventing oscillations
    const double radialDampingTime = (m_props.radialDampingCoeff > 0.0)
        ? std::max(0.008, m_props.carcassMass * 2.0 / m_props.radialDampingCoeff)
        : 0.012;
    const double prevRadialDeflection = m_state.radialDeformation;
    const double dRadial = (targetRadialDeflection - prevRadialDeflection);
    const double maxRate = 12.0; // max 12 m/s radial velocity
    m_state.radialDeformationVelocity = clamp(dRadial / radialDampingTime, -maxRate, maxRate);
    m_state.radialDeformation = clamp(
        prevRadialDeflection + m_state.radialDeformationVelocity * dt,
        0.0,
        maxRadialDeflection
    );

    // Sidewall outward bulge delta_w = nu * delta_r
    m_state.sidewallBulge = m_props.poissonRatio * m_state.radialDeformation;

    // Kinematic rolling radius: R_eff = R_0 - 0.33 * delta_r
    m_state.effectiveRadius = clamp(r0 - 0.333 * m_state.radialDeformation, rRim + 0.02, r0);

    // Compute contact patch dimensions
    computeContactPatchDimensions(normalLoadRequested, m_state.contactPatchLength, m_state.contactPatchWidth);
    m_state.contactPatchArea = m_state.contactPatchLength * m_state.contactPatchWidth * 0.85; // Elliptic factor

    // 3. Populate distributed contact samples across footprint
    const double halfLen = 0.5 * std::max(0.02, m_state.contactPatchLength);
    const double halfWid = 0.5 * std::max(0.05, m_state.contactPatchWidth);

    int activeCount = 0;
    double sumNormalWeights = 0.0;
    std::vector<double> sampleNormalWeights(m_state.contactSamples.size(), 0.0);

    const Vec3 contactGroundRef = vec3Add(hubCenterWorld, vec3Scale(chassisDown, r0 - m_state.radialDeformation));

    for (size_t k = 0; k < m_state.contactSamples.size(); ++k) {
        TireContactPointSample& s = m_state.contactSamples[k];

        // Offset in tire patch coordinate frame (steeredBasis.forward * u * halfLen, steeredBasis.left * v * halfWid)
        const double uDist = s.normalizedU * halfLen;
        const double vDist = s.normalizedV * halfWid;

        // Position on reference cylindrical tyre outer surface
        const Vec3 patchOffset = vec3Add(
            vec3Scale(steeredBasis.forward, uDist),
            vec3Scale(steeredBasis.left, vDist)
        );
        s.undeformedPosWorld = vec3Add(vec3Add(hubCenterWorld, vec3Scale(chassisDown, r0)), patchOffset);

        // Ground contact point on flattened deformed patch
        s.contactPointWorld = vec3Add(contactGroundRef, patchOffset);
        s.contactNormalWorld = vec3Scale(chassisDown, -1.0); // Facing upward into tyre

        // Local penetration depth: maximum near center of patch, tapering smoothly toward edges
        // (Smooth parabolic pressure distribution across patch)
        const double uWeight = 0.20 + 0.80 * std::max(0.0, 1.0 - s.normalizedU * s.normalizedU);
        const double vWeight = 0.20 + 0.80 * std::max(0.0, 1.0 - s.normalizedV * s.normalizedV);
        const double patchWeight = uWeight * vWeight;

        if (normalLoadRequested > 5.0) {
            s.inContact = true;
            s.penetrationDepth = m_state.radialDeformation * patchWeight;
            s.penetrationVelocity = m_state.radialDeformationVelocity * patchWeight;
            sampleNormalWeights[k] = patchWeight;
            sumNormalWeights += patchWeight;
            activeCount++;
        } else {
            s.inContact = false;
            s.penetrationDepth = 0.0;
            s.penetrationVelocity = 0.0;
            s.normalLoad = 0.0;
            s.longitudinalForce = 0.0;
            s.lateralForce = 0.0;
            s.slidingVelocityMps = 0.0;
            s.frictionalPowerWatts = 0.0;
        }
    }

    m_state.activeContactSamples = activeCount;

    // 4. Distribute normal load and compute local shear forces across active samples
    double totalFz = 0.0;
    double totalFx = 0.0;
    double totalFy = 0.0;
    double aligningMoment = 0.0;
    double totalFricPower = 0.0;
    Vec3 centroidSum{0.0, 0.0, 0.0};

    if (activeCount > 0 && sumNormalWeights > 1e-6) {
        const double invSumWeights = 1.0 / sumNormalWeights;

        // Overall tire longitudinal and lateral slip velocities
        const double treadSpeed = wheelAngularVelocity * m_state.effectiveRadius;
        const double slipVx = treadSpeed - chassisVx;
        const double slipVy = -chassisVy;

        for (size_t k = 0; k < m_state.contactSamples.size(); ++k) {
            TireContactPointSample& s = m_state.contactSamples[k];
            if (!s.inContact) continue;

            const double wFraction = sampleNormalWeights[k] * invSumWeights;
            s.normalLoad = normalLoadRequested * wFraction;
            totalFz += s.normalLoad;

            // Local sliding velocity (accounting for angular velocity and patch location)
            const double localVx = slipVx;
            const double localVy = slipVy;
            s.slidingVelocityMps = std::hypot(localVx, localVy);

            // Local friction coefficient with thermal and wear scaling
            const double baseMu = surface.frictionMultiplier * m_state.thermalGripMultiplier * (1.0 - 0.20 * m_state.wearLevel);
            s.frictionCoeff = baseMu;

            // Local shear traction forces
            const double maxTraction = s.normalLoad * s.frictionCoeff;
            const double denomV = std::max(1.0, std::abs(chassisVx));
            const double localKappa = clamp((treadSpeed - chassisVx) / denomV, -1.5, 1.5);
            const double localAlpha = clamp(std::atan2(localVy, denomV), -1.2, 1.2);
            s.longitudinalSlip = localKappa;
            s.lateralSlip = localAlpha;

            // Longitudinal traction force
            const double cLong = m_state.longitudinalStiffness * 0.08;
            const double rawFx = clamp(cLong * localKappa * s.normalLoad / 1000.0, -maxTraction, maxTraction);
            s.longitudinalForce = rawFx;

            // Lateral cornering force
            const double cLat = m_state.lateralStiffness * 0.07;
            const double rawFy = clamp(cLat * localAlpha * s.normalLoad / 1000.0, -maxTraction, maxTraction);
            s.lateralForce = rawFy;

            // Combined slip friction circle constraint per contact point
            const double forceMag = std::hypot(rawFx, rawFy);
            if (forceMag > maxTraction && forceMag > 1e-4) {
                const double scale = maxTraction / forceMag;
                s.longitudinalForce *= scale;
                s.lateralForce *= scale;
            }

            totalFx += s.longitudinalForce;
            totalFy += s.lateralForce;

            // Pneumatic trail self-aligning moment: M_z = -t_p * F_y
            // In the brush model, lateral force centroid is shifted rearward of patch center by t_p ~= 0.22 * halfLen
            const double momentArmLong = (s.normalizedU - 0.22) * halfLen;
            aligningMoment -= s.lateralForce * momentArmLong;

            // Local frictional power: P_j = |F_j * v_slip|
            s.frictionalPowerWatts = std::abs(s.longitudinalForce * localVx) + std::abs(s.lateralForce * localVy);
            totalFricPower += s.frictionalPowerWatts;

            centroidSum = vec3Add(centroidSum, vec3Scale(s.contactPointWorld, wFraction));
        }

        m_state.contactCentroidWorld = centroidSum;
    } else {
        m_state.contactCentroidWorld = contactGroundRef;
    }

    m_state.totalNormalLoad = totalFz;
    m_state.totalLongitudinalForce = totalFx;
    m_state.totalLateralForce = totalFy;
    m_state.selfAligningTorque = aligningMoment;
    m_state.totalFrictionalPowerWatts = totalFricPower;

    // 5. Cyclic structural flexure hysteresis power (proportional to F_z, speed, and deflection):
    // P_flex = eta_loss * F_z * |omega| * delta_r
    const double omegaAbs = std::abs(wheelAngularVelocity);
    m_state.totalFlexurePowerWatts = m_props.hysteresisLossFraction * totalFz * omegaAbs * m_state.radialDeformation;

    // 6. Dynamic Carcass Lateral & Longitudinal Deflections:
    // Under cornering force Fy, carcass deflects laterally: delta_y = Fy / k_y
    // Under braking/drive force Fx, carcass shears longitudinally: delta_x = Fx / k_x
    const double targetDeltaY = clamp(totalFy / m_state.lateralStiffness, -0.06, 0.06);
    const double targetDeltaX = clamp(totalFx / m_state.longitudinalStiffness, -0.05, 0.05);

    const double latTau = 0.015;
    const double longTau = 0.015;
    m_state.lateralDeformation += (targetDeltaY - m_state.lateralDeformation) * clamp(dt / latTau, 0.0, 1.0);
    m_state.longitudinalDeformation += (targetDeltaX - m_state.longitudinalDeformation) * clamp(dt / longTau, 0.0, 1.0);

    // 7. Update thermal and wear state
    const double speedMag = std::hypot(chassisVx, chassisVy);
    updateThermodynamics(dt, speedMag);
    updateWear(dt);
}

} // namespace torquelab
