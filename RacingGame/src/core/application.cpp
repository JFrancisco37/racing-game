#include "Application.h"
#include <cstdio>
#include <cstdlib>
#include <glad/glad.h>
#include "utils.h"
#include "matrices.h"
#include "ObjModel.h"
#include "ShaderManager.h"


// Static pointer for callback routing
static Application* g_app_instance = nullptr;

Application::Application(int argc_, char* argv_[])
    : window(nullptr), argc(argc_), argv(argv_) {
    g_app_instance = this;
}

Application::~Application() {
    cleanup();
}

void Application::run() {
    initGLFW();
    initOpenGL();
    initWindow();
    initCallbacks();
    loadResources();
    mainLoop();
    cleanup();
}

void Application::initGLFW() {
    if (!glfwInit()) {
        fprintf(stderr, "ERROR: glfwInit() failed.\n");
        std::exit(EXIT_FAILURE);
    }
    glfwSetErrorCallback([](int error, const char* description) {
        fprintf(stderr, "GLFW ERROR: %s\n", description);
    });
}

void Application::initOpenGL() {
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
}

void Application::initWindow() {
    window = glfwCreateWindow(800, 600, "Car Racing", NULL, NULL);
    if (!window) {
        glfwTerminate();
        fprintf(stderr, "ERROR: glfwCreateWindow() failed.\n");
        std::exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);

    glfwSetFramebufferSizeCallback(window, FramebufferSizeCallback);
    onFramebufferSize(800, 600);
}

void Application::initCallbacks() {
    glfwSetKeyCallback(window, KeyCallback);
}

void Application::loadResources() {
    // Info da GPU
    printf("GPU: %s, %s, OpenGL %s, GLSL %s\n",
           glGetString(GL_VENDOR),
           glGetString(GL_RENDERER),
           glGetString(GL_VERSION),
           glGetString(GL_SHADING_LANGUAGE_VERSION));

    ShaderManager shaderManager;
    GLuint programID = shaderManager.loadShaders("../../src/shader_vertex.glsl", "../../src/shader_fragment.glsl");

    GLint model_uniform = glGetUniformLocation(programID, "model");
    GLint view_uniform = glGetUniformLocation(programID, "view");
    GLint projection_uniform = glGetUniformLocation(programID, "projection");
    GLint object_id_uniform = glGetUniformLocation(programID, "object_id");

    ObjModel carModel("../../data/car/7LGJ3BYGGGCNZG8ESB4RJZJS0.obj");
    carModel.ComputeNormals();
    BuildTrianglesAndAddToVirtualScene(&carModel);

    ObjModel planeModel("../../data/plane.obj");
    planeModel.ComputeNormals();
    BuildTrianglesAndAddToVirtualScene(&planeModel);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
}

void Application::mainLoop() {
    float oldTime = (float)glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        // Calcular deltaTime
        float currentTime = (float)glfwGetTime();
        float deltaTime = currentTime - oldTime;
        oldTime = currentTime;

        // Atualizar controle do carro baseado na entrada do usuário
        car.throttleInput = front ? 1.0f : 0.0f;
        car.brakeInput = back ? 1.0f : 0.0f;

        if (left) {
            car.steerAngle(-0.5f);  // Esquerda
        } else if (right) {
            car.steerAngle(0.5f);   // Direita
        } else {
            car.steerAngle(0.0f);   // Sem esterçamento
        }

        // Atualizar física do carro
        car.update(deltaTime);

        // Criar a matriz de view com base na posição e orientação do carro
        glm::mat4 view = camera.getViewMatrix(car.position, car.getCarHeading());

        // Criar a matriz de projeção (perspectiva ou ortográfica)
        glm::mat4 projection;
        float nearPlane = -0.1f;
        float farPlane = -10.0f;

        if (usePerspectiveProjection) {
            float fov = 3.141592f / 3.0f;  // 60 graus
            projection = Matrix_Perspective(fov, screenRatio, nearPlane, farPlane);
        } else {
            float t = 1.5f * camera.getDistance() / 2.5f;
            float b = -t;
            float r = t * screenRatio;
            float l = -r;
            projection = Matrix_Orthographic(l, r, b, t, nearPlane, farPlane);
        }

        // Renderizar o frame
        renderer.renderFrame(view, projection, car.getPosition(), car.getCarHeading());

        // Troca dos buffers e captura eventos
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

void Application::cleanup() {
    glfwTerminate();
}

void Application::FramebufferSizeCallback(GLFWwindow* window, int width, int height) {
    if (g_app_instance) g_app_instance->onFramebufferSize(width, height);
}

void Application::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (g_app_instance) g_app_instance->onKey(key, scancode, action, mods);
}

void Application::onFramebufferSize(int width, int height) {
    glViewport(0, 0, width, height);
    screenRatio = (float)width / height;
}

void Application::onKey(int key, int, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (key == GLFW_KEY_W) front = (action != GLFW_RELEASE);
    if (key == GLFW_KEY_S) back = (action != GLFW_RELEASE);
    if (key == GLFW_KEY_A) left = (action != GLFW_RELEASE);
    if (key == GLFW_KEY_D) right = (action != GLFW_RELEASE);

    if (key == GLFW_KEY_P && action == GLFW_PRESS) usePerspectiveProjection = true;
    if (key == GLFW_KEY_O && action == GLFW_PRESS) usePerspectiveProjection = false;
}
