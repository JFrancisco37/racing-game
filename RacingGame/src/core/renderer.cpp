#include "Renderer.h"
#include "matrices.h"
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <cstdio>

// Declare extern se você ainda não encapsulou isso em uma classe:
extern GLuint g_GpuProgramID;
extern GLint g_model_uniform;
extern GLint g_view_uniform;
extern GLint g_projection_uniform;
extern GLint g_object_id_uniform;
extern void DrawVirtualObject(const char* object_name);

Renderer::Renderer()
{
    // Se você precisar inicializar algo, pode fazer aqui
}

void Renderer::renderFrame(const glm::mat4& view, const glm::mat4& projection, const glm::vec4& car_position, const glm::vec4& car_direction)
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(g_GpuProgramID);

    // Envia matrizes para a GPU
    glUniformMatrix4fv(g_view_uniform, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(g_projection_uniform, 1, GL_FALSE, glm::value_ptr(projection));

    // Sphere (carro)
    glm::mat4 model = Matrix_Translate(car_position.x, car_position.y, car_position.z);
    glUniformMatrix4fv(g_model_uniform, 1, GL_FALSE, glm::value_ptr(model));
    glUniform1i(g_object_id_uniform, 0);
    DrawVirtualObject("the_car");

    // Plane
    model = Matrix_Scale(1000.0f, 1.0f, 1000.0f) * Matrix_Translate(0.0f, -1.0f, 0.0f);
    glUniformMatrix4fv(g_model_uniform, 1, GL_FALSE, glm::value_ptr(model));
    glUniform1i(g_object_id_uniform, 2);
    DrawVirtualObject("the_plane");

    // Bunny
    model = Matrix_Translate(1.0f, 0.0f, 0.0f);
    glUniformMatrix4fv(g_model_uniform, 1, GL_FALSE, glm::value_ptr(model));
    glUniform1i(g_object_id_uniform, 1);
    DrawVirtualObject("the_bunny");
}
