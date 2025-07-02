#ifndef CAR_HPP
#define CAR_HPP

#include <glm/glm.hpp>
#include <vector> // For torque curve lookup
#include <string> // For debugging output (optional)

// Define some physics constants if not globally defined elsewhere
const float GRAVITY = 9.81f; // m/s^2

// Struct to represent a point on the torque curve
struct TorquePoint {
    float rpm;
    float torqueNm;
};

class Car {
public:
    // --- State Variables ---
    glm::vec4 position;     // Car's position in world space (x, y, z, 1.0)
    glm::vec4 velocity;     // Car's velocity in world space (vx, vy, vz, 0.0)
    glm::vec4 acceleration; // Car's linear acceleration in world space (ax, ay, az, 0.0)

    float yawAngle;         // Car's rotation around the up-axis (Y-axis), in radians
    float yawRate;          // Car's angular velocity around the up-axis (rad/s)
    float yawAcceleration;  // Car's angular acceleration around the up-axis (rad/s^2)

    float wheelAngularVelocity; // Angular velocity of drive wheels (rad/s)
                                // (Simplified: assuming both drive wheels have same angular velocity)

    // --- Physical Parameters ---
    float mass;                 // Car mass (kg)
    float inertia;              // Car's moment of inertia about the yaw axis (kg.m^2)
    float wheelInertia;         // Inertia of one drive wheel (kg.m^2) (simplified to one value for both)

    float halfWidth;            // Half of the car's width (distance from CG to side wheels)
    float wheelbase;            // Distance between front and rear axles (L)
    float cgToFrontAxle;        // Distance from Center of Gravity (CG) to front axle (b)
    float cgToRearAxle;         // Distance from Center of Gravity (CG) to rear axle (c)
    float cgHeight;             // Height of the Center of Gravity (h)

    float wheelRadius;          // Radius of the wheels (Rw)

    // --- Aerodynamics & Rolling Resistance Constants ---
    float C_drag;               // Air resistance constant (0.5 * Cd * A * rho)
    float C_rr;                 // Rolling resistance constant

    // --- Tire Constants (Simplified Linear Model) ---
    float C_traction;           // Traction constant (slope of longitudinal force vs. slip ratio)
    float C_cornering_stiffness;// Cornering stiffness (slope of lateral force vs. slip angle)
    float friction_coefficient; // mu, friction limit of the tire (e.g., 1.0 for street, 1.5 for racing)

    // --- Engine & Transmission Parameters ---
    float engineMaxTorque;       // Max torque (N.m) at a specific RPM (used for simplified engine model)
    float engineMaxRPM;          // Max RPM for engine operation
    float engineIdleRPM;         // Min RPM for engine operation (to prevent stall)
    std::vector<TorquePoint> torqueCurve; // Actual engine torque curve (rpm, torque)

    std::vector<float> gearRatios; // Ratios for each gear (g1, g2, ...)
    float differentialRatio;       // xd
    float transmissionEfficiency;  // n (e.g., 0.7)

    // --- User Input / Control Variables ---
    float throttleInput;        // 0.0 (no throttle) to 1.0 (full throttle)
    float brakeInput;           // 0.0 (no brake) to 1.0 (full brake)
    float steerAngle;           // Steering angle of front wheels (in radians)

    // --- Internal Calculation Variables (can be reset per update if needed) ---
    glm::vec4 totalForce;       // Sum of all forces acting on the car in world space
    float totalYawTorque;       // Sum of all torques around the yaw axis

    // --- Debug/Display Values (optional) ---
    float currentEngineRPM;
    float currentLongitudinalForce;
    float currentLateralForceFront;
    float currentLateralForceRear;
    float currentSlipRatio;
    float currentSlipAngleFront;
    float currentSlipAngleRear;
    float currentBrakingForce;


    // --- Constructors ---
    Car();
    Car(glm::vec4 initialPosition, glm::vec4 initialVelocity, float initialYawAngle = 0.0f);

    // --- Methods ---
    void update(float deltaTime);

    // Helper functions for torque curve lookup and setup
    void setupDefaultCorvetteParameters();
    float lookupTorque(float rpm);

private:
    // Helper function to get the car's heading vector from its yaw angle
    glm::vec4 getCarHeading() const;
    // Helper function to get the car's side vector from its yaw angle
    glm::vec4 getCarSide() const;
};

#endif // CAR_HPP
