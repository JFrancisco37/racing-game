#ifndef CAR_HPP
#define CAR_HPP

#include <glm/glm.hpp>

class Car {
public:
    glm::vec4 position;
    glm::vec4 velocity;
    glm::vec4 acceleration;

    Car();
    Car(const glm::vec4& initialPosition, const glm::vec4& initialVelocity, const glm::vec4& initialAcceleration);

    void update(float deltaTime);
};

#endif // CAR_HPP
