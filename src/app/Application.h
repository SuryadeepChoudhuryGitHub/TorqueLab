#pragma once
/**
 * Main Native Application Coordinator (GLFW + OpenGL 3.3 Core + Bullet + Dear ImGui).
 */

#include <memory>
#include <string>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "input/InputControllerGLFW.h"
#include "physics/CollisionMeshGeometry.h"
#include "physics/PhysicsWorld.h"
#include "renderer/CameraControllerGL.h"
#include "renderer/DebugOverlayGL.h"
#include "renderer/Shader.h"
#include "renderer/ShadowMapFBO.h"
#include "renderer/TrackRendererGL.h"
#include "renderer/VehicleRendererGL.h"
#include "ui/ImGuiDashboard.h"

namespace torquelab {

class Application {
public:
    Application() = default;
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    bool init(int width = 1440, int height = 900, bool visible = true);
    int run(int maxFrames = 0);
    std::string runValidationSuite();
    void switchActiveVehicle(const std::string& vehicleId);

private:
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void joystickCallback(int jid, int event);

    void renderFrame(double alpha);
    void shutdown();

    GLFWwindow* m_window{nullptr};
    bool m_imguiInitialized{false};
    int m_fbWidth{1440};
    int m_fbHeight{900};
    int m_currentPresetIndex{0};

    bool m_mouseDragging{false};
    double m_prevMouseX{0.0};
    double m_prevMouseY{0.0};

    std::unique_ptr<PhysicsWorld> m_physicsWorld;
    Shader m_mainShader;
    Shader m_depthShader;
    Shader m_lineShader;
    ShadowMapFBO m_shadowFBO;
    TrackRendererGL m_trackRenderer;
    VehicleRendererGL m_vehicleRenderer;
    DebugOverlayGL m_debugOverlay;
    std::unique_ptr<CameraControllerGL> m_cameraController;
    std::unique_ptr<InputControllerGLFW> m_inputController;
    std::unique_ptr<ImGuiDashboard> m_dashboard;
    GeneratedVehicleCollision m_bmwCollision;
};

} // namespace torquelab
