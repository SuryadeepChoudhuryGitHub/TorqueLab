#include <cstdio>
#include <iostream>
#include <string>
#include "app/Application.h"
#include "vehicle/Drivetrain.h"

namespace {

void runFullPipelineDiagnostics() {
    using namespace torquelab;
    const VehicleConfig cfg = createDefaultVehicleConfig();
    PhysicsWorld sim(cfg, true); // Actual Proving Ground used in Application
    for (int i = 0; i < 60; ++i) sim.stepFixed();

    std::printf("========================================================================================================================\n");
    std::printf(" TORQUE LAB NATIVE — FULL PIPELINE ACCELERATION DIAGNOSTICS (dt = %.5f s, %.1f Hz)\n", sim.fixedDt, 1.0 / sim.fixedDt);
    std::printf("========================================================================================================================\n");
    std::printf("Time(s) | Thr | Brk | Gear(Rat) |  RPM (Lim) | EngTrq | WhlTrq(RL) | w_RL(r/s) | Slip_k |  Fz_Rear | GripMax |  Fx_Drive | F_Drag |   F_RR |   a_x  |  v(m/s) | v(km/h) | PosZ(m)\n");
    std::printf("--------+-----+-----+-----------+------------+--------+------------+-----------+--------+----------+---------+-----------+--------+--------+--------+---------+---------+--------\n");

    sim.vehicle->currentInputs = VehicleControlInputs{1.0, 0.0, 0.0, 0.0, false};

    for (int step = 0; step <= 1200; ++step) {
        if (step > 0) {
            sim.stepFixed();
        }
        if (step % 30 == 0 || step <= 15) {
            const auto snap = sim.vehicle->getTelemetrySnapshot();
            const auto& rl = snap.wheels[2];
            const auto& rr = snap.wheels[3];
            const auto& fl = snap.wheels[0];
            const auto& fr = snap.wheels[1];
            const double totRatio = getTotalGearRatio(snap.drivetrain.gearIndex, cfg);
            const double fzRear = rl.normalLoad + rr.normalLoad;
            const double gripMaxRear = rl.maxAvailableGripForce + rr.maxAvailableGripForce;
            const double fxDrive = fl.longitudinalForce + fr.longitudinalForce + rl.longitudinalForce + rr.longitudinalForce;
            double fRR = 0.0;
            for (const auto& w : snap.wheels) {
                if (w.inContact && w.normalLoad > 0.0) {
                    const double crr = cfg.rollingResistanceCoeff * w.surface.rollingResistanceMultiplier;
                    fRR += -crr * w.normalLoad * clamp(w.patchSpeedLongitudinal / 0.4, -1.0, 1.0);
                }
            }
            std::printf(
                "%7.3f | %3.1f | %3.1f | %2s (%5.2f)| %6.0f (%c) | %6.1f | %10.1f | %9.2f | %+6.2f | %8.0f | %7.0f | %9.1f | %6.1f | %6.1f | %+6.2f | %7.2f | %7.1f | %7.2f\n",
                snap.simTime - 0.5,
                sim.vehicle->currentInputs.throttle,
                sim.vehicle->currentInputs.brake,
                snap.drivetrain.gearLabel.c_str(),
                totRatio,
                snap.drivetrain.rpm,
                snap.drivetrain.revLimiterActive ? 'Y' : 'N',
                snap.drivetrain.engineTorqueNm,
                rl.driveTorque,
                rl.angularVelocity,
                rl.slipRatio,
                fzRear,
                gripMaxRear,
                fxDrive,
                snap.aero.dragMagnitude,
                fRR,
                snap.longitudinalAccelMps2,
                snap.forwardSpeedMps,
                snap.speedKmh,
                snap.chassisPosition.z
            );
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    bool smokeTest = false;
    bool runDiag = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--smoke-test") {
            smokeTest = true;
        } else if (arg == "--diag") {
            runDiag = true;
        }
    }

    torquelab::Application app;
    if (runDiag) {
        runFullPipelineDiagnostics();
        if (app.init(1440, 900, true)) {
            app.run(600);
        }
        return 0;
    }

    if (!app.init(1440, 900, !smokeTest)) {
        return 1;
    }

    if (smokeTest) {
        const std::string report = app.runValidationSuite();
        std::cout << "[TorqueLabNative Smoke Test] " << report << std::endl;
        return app.run(15);
    }

    return app.run(0);
}
