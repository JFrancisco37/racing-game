#ifndef RENDERER_H
#define RENDERER_H

#include <glm/mat4x4.hpp>
#include <GLFW/glfw3.h>

class Renderer
{
public:
    Renderer();
    void renderFrame(const glm::mat4& view, const glm::mat4& projection, const glm::vec4& car_position, const glm::vec4& car_direction);
};

#endif // RENDERER_H
