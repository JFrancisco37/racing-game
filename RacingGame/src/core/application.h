#ifndef APPLICATION_H
#define APPLICATION_H

#include <GLFW/glfw3.h>
#include <glm/mat4x4.hpp>
#include "Car.hpp"
#include "Camera.h"
#include "Renderer.h"

class Application {
public:
    Application(int argc, char* argv[]);
    ~Application();

    void run();

private:
    // Inicialização
    void initGLFW();
    void initOpenGL();
    void initWindow();
    void initCallbacks();
    void loadResources();

    // Loop principal e encerramento
    void mainLoop();
    void cleanup();

    // Callbacks
    static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    
    void onFramebufferSize(int width, int height);
    void onKey(int key, int scancode, int action, int mods);

private:
    GLFWwindow* window;
    int argc;
    char** argv;
    float screenRatio = 1.0f;
    bool usePerspectiveProjection = true;

    Car car;
    Camera camera;
    Renderer renderer;
};

#endif // APPLICATION_H
