#include "Car.hpp"

Car::Car()
    : position(0.0f, 0.0f, 0.0f, 1.0f),
      velocity(0.0f, 0.0f, 0.0f, 0.0f),
      acceleration(0.0f, 0.0f, 0.0f, 0.0f)
{
}

Car::Car(const glm::vec4& initialPosition, const glm::vec4& initialVelocity, const glm::vec4& initialAcceleration)
    : position(initialPosition),
      velocity(initialVelocity),
      acceleration(initialAcceleration)
{
    position.w = 1.0f;
    velocity.w = 0.0f;
    acceleration.w = 0.0f;
}

void Car::update(float deltaTime) {
    velocity += acceleration * deltaTime;
    position += velocity * deltaTime;
}
