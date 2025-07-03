#ifndef SHADERMANAGER_H
#define SHADERMANAGER_H

#include <string>
#include <glad/glad.h>

class ShaderManager
{
public:
    ShaderManager();
    ~ShaderManager();

    GLuint loadShaders(const std::string& vertex_path, const std::string& fragment_path);

private:
    GLuint loadShaderFromFile(const std::string& path, GLenum shader_type);
};

#endif // SHADERMANAGER_H
