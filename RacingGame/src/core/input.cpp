#include "Input.h"
#include "matrices.h" // Para crossproduct e norm
#include <cstdio>
#include <cstdlib>
#include <limits>

bool front = false;
bool back = false;
bool left = false;
bool right = false;

glm::vec4 car_position = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
glm::vec4 car_direction = glm::vec4(0.0f, 0.0f, -1.0f, 1.0f) - car_position;
glm::vec4 car_steer = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);

// Variáveis globais para o controle da câmera e mouse
bool g_LeftMouseButtonPressed = false;
bool g_RightMouseButtonPressed = false;
bool g_MiddleMouseButtonPressed = false;
double g_LastCursorPosX, g_LastCursorPosY;
float g_CameraTheta = 0.0f;
float g_CameraPhi = 0.0f;
float g_CameraDistance = 3.5f;
float g_ForearmAngleZ = 0.0f;
float g_ForearmAngleX = 0.0f;
float g_TorsoPositionX = 0.0f;
float g_TorsoPositionY = 0.0f;
bool g_UsePerspectiveProjection = true;
bool g_ShowInfoText = true;
float g_AngleX = 0.0f;
float g_AngleY = 0.0f;
float g_AngleZ = 0.0f;

void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mod)
{
    for (int i = 0; i < 10; ++i)
        if (key == GLFW_KEY_0 + i && action == GLFW_PRESS && mod == GLFW_MOD_SHIFT)
            std::exit(100 + i);

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);

    if (key == GLFW_KEY_W) front = (action != GLFW_RELEASE);
    if (key == GLFW_KEY_S) back = (action != GLFW_RELEASE);
    if (key == GLFW_KEY_A) left = (action != GLFW_RELEASE);
    if (key == GLFW_KEY_D) right = (action != GLFW_RELEASE);

    float delta = 3.141592f / 16;

    if (key == GLFW_KEY_X && action == GLFW_PRESS)
        g_AngleX += (mod & GLFW_MOD_SHIFT) ? -delta : delta;
    if (key == GLFW_KEY_Y && action == GLFW_PRESS)
        g_AngleY += (mod & GLFW_MOD_SHIFT) ? -delta : delta;
    if (key == GLFW_KEY_Z && action == GLFW_PRESS)
        g_AngleZ += (mod & GLFW_MOD_SHIFT) ? -delta : delta;

    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
    {
        g_AngleX = g_AngleY = g_AngleZ = 0.0f;
        g_ForearmAngleX = g_ForearmAngleZ = 0.0f;
        g_TorsoPositionX = g_TorsoPositionY = 0.0f;
    }

    if (key == GLFW_KEY_P && action == GLFW_PRESS)
        g_UsePerspectiveProjection = true;
    if (key == GLFW_KEY_O && action == GLFW_PRESS)
        g_UsePerspectiveProjection = false;
    if (key == GLFW_KEY_H && action == GLFW_PRESS)
        g_ShowInfoText = !g_ShowInfoText;
}

void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        g_LeftMouseButtonPressed = (action == GLFW_PRESS);
        glfwGetCursorPos(window, &g_LastCursorPosX, &g_LastCursorPosY);
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        g_RightMouseButtonPressed = (action == GLFW_PRESS);
        glfwGetCursorPos(window, &g_LastCursorPosX, &g_LastCursorPosY);
    }
    if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    {
        g_MiddleMouseButtonPressed = (action == GLFW_PRESS);
        glfwGetCursorPos(window, &g_LastCursorPosX, &g_LastCursorPosY);
    }
}

void CursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
    float dx = xpos - g_LastCursorPosX;
    float dy = ypos - g_LastCursorPosY;

    if (g_LeftMouseButtonPressed)
    {
        g_CameraTheta -= 0.01f * dx;
        g_CameraPhi += 0.01f * dy;
        g_CameraPhi = std::max(std::min(g_CameraPhi, 3.141592f/2), -3.141592f/2);
    }

    if (g_RightMouseButtonPressed)
    {
        g_ForearmAngleZ -= 0.01f * dx;
        g_ForearmAngleX += 0.01f * dy;
    }

    if (g_MiddleMouseButtonPressed)
    {
        g_TorsoPositionX += 0.01f * dx;
        g_TorsoPositionY -= 0.01f * dy;
    }

    g_LastCursorPosX = xpos;
    g_LastCursorPosY = ypos;
}

void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    g_CameraDistance -= 0.1f * yoffset;
    const float min_dist = std::numeric_limits<float>::epsilon();
    if (g_CameraDistance < min_dist)
        g_CameraDistance = min_dist;
}
