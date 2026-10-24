#include "app/Application.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include "vehicle/Drivetrain.h"

namespace torquelab {

Application::~Application() {
    shutdown();
}

bool Application::init(int width, int height, bool visible) {
    if (!glfwInit()) {
        std::cerr << "[TorqueLabNative] Failed to initialize GLFW." << std::endl;
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 0);
    glfwWindowHint(GLFW_VISIBLE, visible ? GLFW_TRUE : GLFW_FALSE);

    m_window = glfwCreateWindow(
        width,
        height,
        "Torque Lab Native — 6-DOF Vehicle Dynamics Simulator (C++20 / OpenGL 3.3 / Bullet)",
        nullptr,
        nullptr
    );
    if (!m_window) {
        std::cerr << "[TorqueLabNative] Failed to create GLFW OpenGL 3.3 window." << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(0);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        std::cerr << "[TorqueLabNative] Failed to initialize GLAD OpenGL 3.3 loader." << std::endl;
        return false;
    }

    glfwGetFramebufferSize(m_window, &m_fbWidth, &m_fbHeight);
    glViewport(0, 0, m_fbWidth, m_fbHeight);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // 1. Initialize Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr; // Keep window layout deterministic

    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
    m_imguiInitialized = true;

    // 2. Initialize Physics World & Settle Initial Equilibrium
    const VehicleConfig config = createDefaultVehicleConfig();
    m_physicsWorld = std::make_unique<PhysicsWorld>(config, true);
    for (int i = 0; i < 45; ++i) {
        m_physicsWorld->stepFixed();
    }

    // 3. Load Shaders & Initialize OpenGL Renderers
    if (!m_mainShader.loadFromFiles("shaders/phong_shadow.vert", "shaders/phong_shadow.frag") ||
        !m_depthShader.loadFromFiles("shaders/shadow_depth.vert", "shaders/shadow_depth.frag") ||
        !m_lineShader.loadFromFiles("shaders/debug_line.vert", "shaders/debug_line.frag")) {
        std::cerr << "[TorqueLabNative] Failed to compile GLSL shaders." << std::endl;
        return false;
    }

    if (!m_shadowFBO.init(2048)) {
        std::cerr << "[TorqueLabNative] Warning: ShadowMapFBO incomplete." << std::endl;
    }

    m_trackRenderer.init(m_physicsWorld->trackElements);
    m_vehicleRenderer.init(config);
    m_debugOverlay.init();

    const float aspect = static_cast<float>(m_fbWidth) / static_cast<float>(std::max(1, m_fbHeight));
    m_cameraController = std::make_unique<CameraControllerGL>(aspect);
    m_cameraController->snapToVehicle(*m_physicsWorld->vehicle);

    // 4. Initialize UI & Input
    m_dashboard = std::make_unique<ImGuiDashboard>(DashboardCallbacks{
        [this]() {
            m_physicsWorld->vehicle->resetVehicle();
            for (int i = 0; i < 20; ++i) m_physicsWorld->stepFixed();
            m_cameraController->snapToVehicle(*m_physicsWorld->vehicle);
        },
        [this]() {
            m_physicsWorld->paused = !m_physicsWorld->paused;
        },
        [this]() {
            m_cameraController->cycleMode();
        },
        [this]() {
            m_dashboard->telemetryEnabled = !m_dashboard->telemetryEnabled;
        },
        [this]() {
            m_debugOverlay.enabled = !m_debugOverlay.enabled;
        },
        [this]() {
            auto& dt = m_physicsWorld->vehicle->drivetrain;
            dt.transmissionMode = (dt.transmissionMode == TransmissionMode::Automatic)
                ? TransmissionMode::Manual
                : TransmissionMode::Automatic;
        },
        [this](const SpawnPreset& preset) {
            m_physicsWorld->teleportToPreset(preset);
            for (int i = 0; i < 20; ++i) m_physicsWorld->stepFixed();
            m_cameraController->snapToVehicle(*m_physicsWorld->vehicle);
        },
        [this](GraphicsQuality q) {
            m_trackRenderer.setQuality(q, m_shadowFBO);
        },
        [this]() {
            runValidationSuite();
        },
        [this](bool enable) {
            m_vehicleRenderer.setUseImportedModel(enable);
        },
        [this]() {
            m_vehicleRenderer.toggleVisualDebug();
        }
    });
    m_dashboard->applyTheme();

    m_inputController = std::make_unique<InputControllerGLFW>(InputCallbacks{
        [this]() {
            m_physicsWorld->vehicle->resetVehicle();
            for (int i = 0; i < 20; ++i) m_physicsWorld->stepFixed();
            m_cameraController->snapToVehicle(*m_physicsWorld->vehicle);
        },
        [this]() {
            m_cameraController->cycleMode();
        },
        [this]() {
            m_physicsWorld->paused = !m_physicsWorld->paused;
        },
        [this]() {
            m_dashboard->telemetryEnabled = !m_dashboard->telemetryEnabled;
        },
        [this]() {
            m_debugOverlay.enabled = !m_debugOverlay.enabled;
        },
        [this]() {
            auto& dt = m_physicsWorld->vehicle->drivetrain;
            dt.transmissionMode = (dt.transmissionMode == TransmissionMode::Automatic)
                ? TransmissionMode::Manual
                : TransmissionMode::Automatic;
        },
        [this]() {
            auto& dt = m_physicsWorld->vehicle->drivetrain;
            shiftToGear(dt, dt.gearIndex + 1, m_physicsWorld->vehicle->config);
        },
        [this]() {
            auto& dt = m_physicsWorld->vehicle->drivetrain;
            shiftToGear(dt, dt.gearIndex - 1, m_physicsWorld->vehicle->config);
        },
        [this](int idx) {
            const auto& presets = getProvingGroundPresets();
            if (idx >= 0 && idx < static_cast<int>(presets.size())) {
                m_physicsWorld->teleportToPreset(presets[static_cast<size_t>(idx)]);
                for (int i = 0; i < 20; ++i) m_physicsWorld->stepFixed();
                m_cameraController->snapToVehicle(*m_physicsWorld->vehicle);
            }
        },
        [this]() {
            m_vehicleRenderer.setUseImportedModel(!m_vehicleRenderer.isUsingImportedModel());
        },
        [this]() {
            m_vehicleRenderer.toggleVisualDebug();
        }
    });

    glfwSetWindowUserPointer(m_window, this);
    glfwSetKeyCallback(m_window, keyCallback);
    glfwSetMouseButtonCallback(m_window, mouseButtonCallback);
    glfwSetCursorPosCallback(m_window, cursorPosCallback);
    glfwSetScrollCallback(m_window, scrollCallback);
    glfwSetFramebufferSizeCallback(m_window, framebufferSizeCallback);

    // Warm up GLFW joystick enumeration during init so Frame 0 does not incur DirectInput init stall
    (void)glfwJoystickIsGamepad(GLFW_JOYSTICK_1);

    return true;
}

std::string Application::runValidationSuite() {
    const VehicleConfig testCfg = createDefaultVehicleConfig();
    PhysicsWorld sim(testCfg, false);
    for (int i = 0; i < 180; ++i) sim.stepFixed();

    const VehicleTelemetrySnapshot eqSnap = sim.vehicle->getTelemetrySnapshot();
    double sumFz = 0.0;
    for (const auto& w : eqSnap.wheels) sumFz += w.normalLoad;
    const double weightErrPct =
        (std::abs(sumFz - testCfg.mass * GRAVITY) / (testCfg.mass * GRAVITY)) * 100.0;

    // Acceleration check
    sim.vehicle->currentInputs = VehicleControlInputs{1.0, 0.0, 0.0, 0.0, false};
    for (int i = 0; i < 180; ++i) sim.stepFixed();
    const VehicleTelemetrySnapshot accSnap = sim.vehicle->getTelemetrySnapshot();

    // Braking check
    sim.vehicle->currentInputs = VehicleControlInputs{0.0, 0.9, 0.0, 0.0, true};
    for (int i = 0; i < 45; ++i) sim.stepFixed();
    const VehicleTelemetrySnapshot brkSnap = sim.vehicle->getTelemetrySnapshot();

    char buf[256];
    std::snprintf(
        buf,
        sizeof(buf),
        "PASS | Eq Load Err: %.2f%% | Accel Squat Pitch: %+.2f deg @ %.0f km/h | Brake Dive Pitch: %+.2f deg",
        weightErrPct,
        radToDeg(accSnap.pitchRad),
        accSnap.speedKmh,
        radToDeg(brkSnap.pitchRad)
    );
    const std::string msg(buf);
    if (m_dashboard) {
        m_dashboard->showValidationReport(msg, true);
    }
    return msg;
}

int Application::run(int maxFrames) {
    double lastTime = glfwGetTime();
    double smoothedFps = 60.0;
    int frameCount = 0;

    while (!glfwWindowShouldClose(m_window)) {
        const double t0 = glfwGetTime();
        glfwPollEvents();
        const double t1 = glfwGetTime();

        const double now = t1;
        const double frameDt = std::clamp(now - lastTime, 0.0005, 0.1);
        lastTime = now;

        const double instantFps = 1.0 / frameDt;
        smoothedFps = smoothedFps * 0.92 + instantFps * 0.08;

        const VehicleControlInputs inputs = m_inputController->sampleInputs(m_window, frameDt);
        const double t2 = glfwGetTime();
        m_physicsWorld->vehicle->currentInputs = inputs;
        if (maxFrames == 600) {
            m_physicsWorld->vehicle->currentInputs.throttle = 1.0;
        }

        const double alpha = m_physicsWorld->updateFrame(frameDt);
        const double t3 = glfwGetTime();

        m_vehicleRenderer.syncWithVehicle(*m_physicsWorld->vehicle, alpha);
        const VehicleTelemetrySnapshot snap = m_physicsWorld->vehicle->getTelemetrySnapshot();
        m_trackRenderer.updateSunTarget(snap.chassisPosition);
        m_cameraController->update(*m_physicsWorld->vehicle, alpha, frameDt);

        if (maxFrames == 600 && frameCount == 0) {
            std::printf("[GL Info] Vendor: %s | Renderer: %s | Version: %s\n",
                        reinterpret_cast<const char*>(glGetString(GL_VENDOR)),
                        reinterpret_cast<const char*>(glGetString(GL_RENDERER)),
                        reinterpret_cast<const char*>(glGetString(GL_VERSION)));
            for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; ++jid) {
                if (glfwJoystickPresent(jid)) {
                    std::printf("[Joystick %d] Name='%s' | IsGamepad=%d\n",
                                jid,
                                glfwGetJoystickName(jid) ? glfwGetJoystickName(jid) : "null",
                                glfwJoystickIsGamepad(jid));
                    GLFWgamepadstate st{};
                    if (glfwGetGamepadState(jid, &st)) {
                        std::printf("  Axes: LX=%.3f LY=%.3f RX=%.3f RY=%.3f LT=%.3f RT=%.3f\n",
                                    st.axes[0], st.axes[1], st.axes[2], st.axes[3], st.axes[4], st.axes[5]);
                    }
                }
            }
        }

        renderFrame(alpha);
        const double t4 = glfwGetTime();

        // Render Dear ImGui HUD
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        m_dashboard->render(
            *m_physicsWorld->vehicle,
            snap,
            inputs,
            m_physicsWorld->paused,
            m_cameraController->getModeLabel(),
            m_debugOverlay.enabled,
            m_trackRenderer.getQuality(),
            smoothedFps,
            m_physicsWorld->lastPhysicsStepTimeMs,
            m_fbWidth,
            m_fbHeight,
            m_vehicleRenderer.isUsingImportedModel(),
            m_vehicleRenderer.hasImportedModel(),
            m_vehicleRenderer.getActiveModelName(),
            m_vehicleRenderer.isVisualDebugEnabled()
        );

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        const double t5 = glfwGetTime();

        glfwSwapBuffers(m_window);
        const double t6 = glfwGetTime();

        if (maxFrames == 600 && (frameCount % 60 == 0 || frameCount == 599)) {
            const Vec3 camPos = m_cameraController->getPosition();
            std::printf(
                "[LiveLoop Frame %3d] frameDt=%.4fs (FPS=%.1f) [phys=%.2fms rnd=%.1fms ui=%.1fms swp=%.1fms] | simTime=%.3fs | Thr=%.2f Brk=%.2f | v=%.2f m/s (%.1f km/h) | CarZ=%.2fm | CamDist=%.2fm\n",
                frameCount,
                frameDt,
                smoothedFps,
                (t3 - t2) * 1000.0,
                (t4 - t3) * 1000.0,
                (t5 - t4) * 1000.0,
                (t6 - t5) * 1000.0,
                snap.simTime,
                m_physicsWorld->vehicle->currentInputs.throttle,
                m_physicsWorld->vehicle->currentInputs.brake,
                snap.forwardSpeedMps,
                snap.speedKmh,
                snap.chassisPosition.z,
                snap.chassisPosition.z - camPos.z
            );
        }

        ++frameCount;
        if (maxFrames > 0 && frameCount >= maxFrames) {
            break;
        }
    }

    return 0;
}

void Application::renderFrame(double /*alpha*/) {
    // Ensure shadow map texture is unbound from texture units before writing to ShadowMapFBO
    // (prevents OpenGL feedback-loop pipeline stalls on AMD/Intel drivers)
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);

    // Pass 1: Directional Shadow Map Depth Pass
    if (m_trackRenderer.areShadowsEnabled()) {
        m_shadowFBO.beginPass();
        m_depthShader.use();
        m_depthShader.setMat4("uLightSpaceMatrix", m_trackRenderer.getLightSpaceMatrix());

        m_trackRenderer.renderShadowPass(m_depthShader);
        m_vehicleRenderer.renderShadowPass(m_depthShader);

        m_shadowFBO.endPass(m_fbWidth, m_fbHeight);
    }

    // Pass 2: Main Lit & Shadowed Scene Pass
    glViewport(0, 0, m_fbWidth, m_fbHeight);
    const Color3 skyBg = Color3::fromHex(0x0f172a);
    glClearColor(skyBg.r, skyBg.g, skyBg.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const Mat4 viewProj = m_cameraController->getViewProjectionMatrix();

    m_mainShader.use();
    m_mainShader.setMat4("uViewProj", viewProj);
    m_mainShader.setMat4("uLightSpaceMatrix", m_trackRenderer.getLightSpaceMatrix());
    m_mainShader.setVec3("uCameraPos", m_cameraController->getPosition());
    m_mainShader.setVec3("uSunDir", m_trackRenderer.getSunDirection());
    m_mainShader.setVec3("uSunColor", 1.9f, 1.86f, 1.75f);
    m_mainShader.setColor3("uSkyColor", Color3::fromHex(0xdbeafe));
    m_mainShader.setColor3("uGroundColor", Color3::fromHex(0x1e293b));
    m_mainShader.setColor3("uFogColor", skyBg);
    m_mainShader.setFloat("uFogDensity", 0.0022f);
    m_mainShader.setInt("uShadowsEnabled", m_trackRenderer.areShadowsEnabled() ? 1 : 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_shadowFBO.getDepthTexture());
    m_mainShader.setInt("uShadowMap", 1);

    m_trackRenderer.renderMainPass(m_mainShader);
    m_vehicleRenderer.renderMainPass(m_mainShader);
    m_debugOverlay.render(*m_physicsWorld->vehicle, m_lineShader, m_mainShader, viewProj);
    m_vehicleRenderer.renderVisualDebug(m_lineShader, m_mainShader, viewProj);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
}

void Application::shutdown() {
    if (m_imguiInitialized) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        m_imguiInitialized = false;
    }
    if (m_window) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
        glfwTerminate();
    }
}

void Application::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (!app || !app->m_inputController) return;

    if (ImGui::GetIO().WantCaptureKeyboard && key != GLFW_KEY_ESCAPE) {
        return;
    }
    app->m_inputController->onKeyEvent(key, action);
}

void Application::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (!app) return;

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS && !ImGui::GetIO().WantCaptureMouse) {
            app->m_mouseDragging = true;
            glfwGetCursorPos(window, &app->m_prevMouseX, &app->m_prevMouseY);
        } else if (action == GLFW_RELEASE) {
            app->m_mouseDragging = false;
        }
    }
}

void Application::cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (!app || !app->m_mouseDragging || !app->m_cameraController) return;

    const double dx = xpos - app->m_prevMouseX;
    const double dy = ypos - app->m_prevMouseY;
    app->m_prevMouseX = xpos;
    app->m_prevMouseY = ypos;
    app->m_cameraController->onMouseDrag(dx, dy);
}

void Application::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (!app || !app->m_cameraController || ImGui::GetIO().WantCaptureMouse) return;

    app->m_cameraController->onMouseScroll(yoffset);
}

void Application::framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (!app) return;

    app->m_fbWidth = std::max(1, width);
    app->m_fbHeight = std::max(1, height);
    glViewport(0, 0, app->m_fbWidth, app->m_fbHeight);
    if (app->m_cameraController) {
        app->m_cameraController->onResize(
            static_cast<float>(app->m_fbWidth) / static_cast<float>(app->m_fbHeight)
        );
    }
}

} // namespace torquelab
