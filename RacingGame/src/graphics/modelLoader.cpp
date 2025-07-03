#include "modelLoader.h"
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <cassert>
#include <cmath>

void ComputeNormals(ObjModel* model) {
    if (!model || !model->attrib.normals.empty())
        return;

    size_t num_vertices = model->attrib.vertices.size() / 3;
    std::vector<glm::vec3> vertex_normals(num_vertices, glm::vec3(0.0f));
    std::vector<int> num_triangles_per_vertex(num_vertices, 0);

    for (const auto& shape : model->shapes) {
        size_t num_faces = shape.mesh.num_face_vertices.size();
        for (size_t f = 0; f < num_faces; ++f) {
            assert(shape.mesh.num_face_vertices[f] == 3);
            glm::vec3 v[3];
            int idx[3];
            for (int i = 0; i < 3; ++i) {
                idx[i] = shape.mesh.indices[3 * f + i].vertex_index;
                v[i] = glm::vec3(
                    model->attrib.vertices[3 * idx[i] + 0],
                    model->attrib.vertices[3 * idx[i] + 1],
                    model->attrib.vertices[3 * idx[i] + 2]
                );
            }
            glm::vec3 normal = glm::normalize(glm::cross(v[1] - v[0], v[2] - v[0]));
            for (int i = 0; i < 3; ++i) {
                vertex_normals[idx[i]] += normal;
                num_triangles_per_vertex[idx[i]]++;
                model->shapes[0].mesh.indices[3 * f + i].normal_index = idx[i];
            }
        }
    }

    model->attrib.normals.resize(3 * num_vertices);
    for (size_t i = 0; i < num_vertices; ++i) {
        if (num_triangles_per_vertex[i] > 0) {
            glm::vec3 n = glm::normalize(vertex_normals[i] / (float)num_triangles_per_vertex[i]);
            model->attrib.normals[3 * i + 0] = n.x;
            model->attrib.normals[3 * i + 1] = n.y;
            model->attrib.normals[3 * i + 2] = n.z;
        }
    }
}

void BuildTrianglesAndAddToVirtualScene(ObjModel* model, std::map<std::string, SceneObject>& virtualScene) {
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    std::vector<GLuint> indices;
    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<float> texcoords;

    for (const auto& shape : model->shapes) {
        size_t first_index = indices.size();
        size_t num_faces = shape.mesh.num_face_vertices.size();

        for (size_t f = 0; f < num_faces; ++f) {
            assert(shape.mesh.num_face_vertices[f] == 3);
            for (int v = 0; v < 3; ++v) {
                tinyobj::index_t idx = shape.mesh.indices[3 * f + v];
                indices.push_back(static_cast<GLuint>(indices.size()));

                positions.push_back(model->attrib.vertices[3 * idx.vertex_index + 0]);
                positions.push_back(model->attrib.vertices[3 * idx.vertex_index + 1]);
                positions.push_back(model->attrib.vertices[3 * idx.vertex_index + 2]);
                positions.push_back(1.0f);

                if (idx.normal_index != -1) {
                    normals.push_back(model->attrib.normals[3 * idx.normal_index + 0]);
                    normals.push_back(model->attrib.normals[3 * idx.normal_index + 1]);
                    normals.push_back(model->attrib.normals[3 * idx.normal_index + 2]);
                    normals.push_back(0.0f);
                }

                if (idx.texcoord_index != -1) {
                    texcoords.push_back(model->attrib.texcoords[2 * idx.texcoord_index + 0]);
                    texcoords.push_back(model->attrib.texcoords[2 * idx.texcoord_index + 1]);
                }
            }
        }

        SceneObject obj;
        obj.name = shape.name;
        obj.first_index = first_index;
        obj.num_indices = indices.size() - first_index;
        obj.rendering_mode = GL_TRIANGLES;
        obj.vertex_array_object_id = vao;
        virtualScene[shape.name] = obj;
    }

    // VBOs
    GLuint vbo_positions;
    glGenBuffers(1, &vbo_positions);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_positions);
    glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(float), positions.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);

    if (!normals.empty()) {
        GLuint vbo_normals;
        glGenBuffers(1, &vbo_normals);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_normals);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(float), normals.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
    }

    if (!texcoords.empty()) {
        GLuint vbo_texcoords;
        glGenBuffers(1, &vbo_texcoords);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_texcoords);
        glBufferData(GL_ARRAY_BUFFER, texcoords.size() * sizeof(float), texcoords.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(2);
    }

    GLuint ebo;
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
}
