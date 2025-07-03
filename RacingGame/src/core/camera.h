#ifndef CAMERA_H
#define CAMERA_H

#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>

class Camera {
public:
    Camera();

    glm::mat4 getViewMatrix(const glm::vec4& car_position, const glm::vec4& car_direction);

    void updateAngles(float dTheta, float dPhi);
    void zoom(float delta);

private:
    float theta;
    float phi;
    float distance;
};

#endif // CAMERA_H
