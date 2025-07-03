#include "Camera.h"
#include "matrices.h"  // Para Matrix_Camera_View

#include <cmath>
#include <limits>

Camera::Camera()
    : theta(0.0f), phi(0.0f), distance(3.5f) {}

glm::mat4 Camera::getViewMatrix(const glm::vec4& car_position, const glm::vec4& car_direction) {
    glm::vec4 camera_position_c = car_position - 5.0f * car_direction + glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
    glm::vec4 camera_view_vector = car_position - camera_position_c;
    glm::vec4 camera_up_vector = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);

    return Matrix_Camera_View(camera_position_c, camera_view_vector, camera_up_vector);
}

void Camera::updateAngles(float dTheta, float dPhi) {
    theta += dTheta;
    phi += dPhi;

    float phimax = 3.141592f / 2;
    float phimin = -phimax;

    if (phi > phimax)
        phi = phimax;
    if (phi < phimin)
        phi = phimin;
}

void Camera::zoom(float delta) {
    distance -= delta;
    const float verysmallnumber = std::numeric_limits<float>::epsilon();
    if (distance < verysmallnumber)
        distance = verysmallnumber;
}
