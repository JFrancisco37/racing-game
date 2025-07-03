#ifndef INPUT_H
#define INPUT_H

#include <GLFW/glfw3.h>
#include <glm/vec4.hpp>

// Variáveis globais (declarações externas)
extern bool front, back, left, right;
extern glm::vec4 car_position;
extern glm::vec4 car_direction;
extern glm::vec4 car_steer;

// Funções de callback
void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
void CursorPosCallback(GLFWwindow* window, double xpos, double ypos);
void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

#endif // INPUT_H
