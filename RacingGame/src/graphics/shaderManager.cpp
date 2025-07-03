#include "ShaderManager.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdlib>

ShaderManager::ShaderManager() {}

ShaderManager::~ShaderManager() {}

GLuint ShaderManager::loadShaders(const std::string& vertex_path, const std::string& fragment_path)
{
    GLuint vertex_shader = loadShaderFromFile(vertex_path, GL_VERTEX_SHADER);
    GLuint fragment_shader = loadShaderFromFile(fragment_path, GL_FRAGMENT_SHADER);

    GLuint program_id = glCreateProgram();
    glAttachShader(program_id, vertex_shader);
    glAttachShader(program_id, fragment_shader);
    glLinkProgram(program_id);

    GLint success;
    glGetProgramiv(program_id, GL_LINK_STATUS, &success);
    if (!success)
    {
        GLint log_length;
        glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &log_length);
        char* log = new char[log_length];
        glGetProgramInfoLog(program_id, log_length, NULL, log);
        std::cerr << "ERROR: Shader program linking failed.\n" << log << "\n";
        delete[] log;
        std::exit(EXIT_FAILURE);
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    return program_id;
}

GLuint ShaderManager::loadShaderFromFile(const std::string& path, GLenum shader_type)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "ERROR: Cannot open shader file: " << path << "\n";
        std::exit(EXIT_FAILURE);
    }

    std::stringstream shader_stream;
    shader_stream << file.rdbuf();
    std::string shader_code = shader_stream.str();
    const char* shader_cstr = shader_code.c_str();

    GLuint shader = glCreateShader(shader_type);
    glShaderSource(shader, 1, &shader_cstr, NULL);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        GLint log_length;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
        char* log = new char[log_length];
        glGetShaderInfoLog(shader, log_length, NULL, log);
        std::cerr << "ERROR: Compilation failed for shader: " << path << "\n" << log << "\n";
        delete[] log;
        std::exit(EXIT_FAILURE);
    }

    return shader;
}
