#include "car.hpp"
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp> // For rotating vectors
#include <glm/gtx/norm.hpp>         // For glm::length2
#include <cmath>                    // For sqrt, M_PI, atan2, etc.
#include <algorithm>                // For std::min, std::max

// Define M_PI if not available (some compilers might not have it by default)
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

Car::Car()
    : position(0.0f, 0.0f, 0.0f, 1.0f),
      velocity(0.0f, 0.0f, 0.0f, 0.0f),
      acceleration(0.0f, 0.0f, 0.0f, 0.0f),
      yawAngle(0.0f),
      yawRate(0.0f),
      yawAcceleration(0.0f),
      wheelAngularVelocity(0.0f),
      throttleInput(0.0f),
      brakeInput(0.0f),
      steerAngle(0.0f)
{
    setupDefaultCorvetteParameters(); // Initialize with default values for easier testing
    // Ensure w-component for position is 1, and for vectors (velocity, acceleration, etc.) is 0
    position.w = 1.0f;
    velocity.w = 0.0f;
    acceleration.w = 0.0f;
    totalForce.w = 0.0f;
}

Car::Car(glm::vec4 initialPosition, glm::vec4 initialVelocity, float initialYawAngle)
    : position(initialPosition),
      velocity(initialVelocity),
      acceleration(0.0f, 0.0f, 0.0f, 0.0f),
      yawAngle(initialYawAngle),
      yawRate(0.0f),
      yawAcceleration(0.0f),
      wheelAngularVelocity(0.0f),
      throttleInput(0.0f),
      brakeInput(0.0f),
      steerAngle(0.0f)
{
    setupDefaultCorvetteParameters();
    position.w = 1.0f;
    velocity.w = 0.0f;
    acceleration.w = 0.0f;
    totalForce.w = 0.0f;
}

// Helper to get car's current heading vector in world space
glm::vec4 Car::getCarHeading() const {
    // Assuming 2D movement (X-Z plane), so yaw is rotation around Y-axis
    // Initial heading is usually (1, 0, 0) or (0, 0, 1) depending on convention
    // Let's assume initial heading is along +X and yaw rotates around +Y
    return glm::vec4(cos(yawAngle), 0.0f, sin(yawAngle), 0.0f);
}

// Helper to get car's current side vector in world space
glm::vec4 Car::getCarSide() const {
    // Perpendicular to heading, pointing to the right if yaw is positive counter-clockwise
    // Assuming heading (cos(yaw), 0, sin(yaw)), side is (-sin(yaw), 0, cos(yaw))
    return glm::vec4(-sin(yawAngle), 0.0f, cos(yawAngle), 0.0f);
}

// Function to look up engine torque from the torque curve
float Car::lookupTorque(float rpm) {
    if (torqueCurve.empty()) {
        return 0.0f; // No torque curve defined
    }

    // Clamp RPM to the defined operating range
    rpm = std::max(rpm, engineIdleRPM);
    rpm = std::min(rpm, engineMaxRPM);

    // Find the two points for interpolation
    size_t i = 0;
    for (i = 0; i < torqueCurve.size() - 1; ++i) {
        if (rpm < torqueCurve[i+1].rpm) {
            break;
        }
    }

    // Handle edge cases: RPM below first point or at/above last point
    if (i >= torqueCurve.size() - 1) {
        return torqueCurve.back().torqueNm;
    }

    // Linear interpolation
    const TorquePoint& p1 = torqueCurve[i];
    const TorquePoint& p2 = torqueCurve[i+1];

    float t = (rpm - p1.rpm) / (p2.rpm - p1.rpm);
    return p1.torqueNm + t * (p2.torqueNm - p1.torqueNm);
}

// Setup realistic parameters for a Corvette C5 (from tutorial)
void Car::setupDefaultCorvetteParameters() {
    mass = 1439.0f; // kg (ignoring driver for now)
    wheelbase = 2.654f; // m (265.4 cm for C5 hardtop)
    // Assuming CG is roughly mid-way and height 1.0m for simplified weight transfer
    cgToFrontAxle = wheelbase / 2.0f; // b
    cgToRearAxle = wheelbase / 2.0f;  // c
    cgHeight = 1.0f;                  // h (estimated for generic car physics)
    halfWidth = 0.9f;                 // half of typical car width (approx 1.8m width)

    // Inertia: Approx. for a rectangular prism. For a car, it's more complex.
    // I_yaw = m * (length^2 + width^2) / 12
    // Let's use a rough estimate from tutorials or common values
    inertia = mass * (wheelbase * wheelbase + (halfWidth * 2.0f) * (halfWidth * 2.0f)) / 12.0f;
    // A more common approximation for sports cars is around 2500 - 3500 kg.m^2.
    // Let's use a more typical value for a sports car to make it feel better:
    inertia = 2800.0f; // kg.m^2

    // Wheel inertia (for one wheel) - 75kg wheel with 0.33m radius (as per tutorial's example for solid cylinder)
    // 75 * 0.33^2 / 2 = 4.08 kg.m^2. We'll simplify and use this value for the whole drive axle.
    wheelInertia = 4.1f * 2.0f; // Approx. for both rear wheels + axle itself.

    wheelRadius = 0.34f; // m (P275/40ZR-18 rear tyres of a Corvette)

    C_drag = 0.4257f; // From tutorial: 0.5 * Cd * A * rho
    C_rr = 12.8f;     // From tutorial: 30 * C_drag (highly doubted by tutorial author)

    // Simplified linear tire constants. Tune these for feel.
    friction_coefficient = 1.0f; // mu (street tyres)
    C_traction = 10000.0f;      // Slope for longitudinal force vs slip ratio
    C_cornering_stiffness = 50000.0f; // Slope for lateral force vs slip angle (N/radian)

    // Engine Torque Curve (LS1 V8 from tutorial, converted to Nm and typical RPMs)
    torqueCurve = {
        {1000, 300}, // N.m (estimated)
        {1500, 400},
        {2000, 440},
        {2500, 448}, // 330 ft lbs = 448 Nm (from tutorial)
        {3000, 460},
        {3500, 470},
        {4000, 475}, // Peak torque 350 lb-ft = 475 N.m @ 4400 rpm (from tutorial)
        {4400, 475}, // Actual peak
        {5000, 460},
        {5600, 440}, // Horsepower peaks here
        {6000, 400},
        {6500, 350} // Redline (estimated beyond tutorial's graph)
    };
    engineIdleRPM = 1000.0f;
    engineMaxRPM = 6500.0f;

    // Gear Ratios (Corvette C5 hardtop manual)
    gearRatios = {2.66f, 1.78f, 1.30f, 1.0f, 0.74f, 0.50f}; // 1st to 6th gear
    differentialRatio = 3.42f;
    transmissionEfficiency = 0.7f; // Guess from tutorial
}

void Car::update(float deltaTime) {
    // Convert velocity to car's reference frame (longitudinal and lateral components)
    glm::vec4 carHeading = getCarHeading();
    glm::vec4 carSide = getCarSide();

    float v_long = glm::dot(velocity, carHeading);
    float v_lat = glm::dot(velocity, carSide);
    float speed = glm::length(glm::vec3(velocity.x, velocity.y, velocity.z)); // Magnitude of 3D velocity

    // Ensure speed is not negative
    if (speed < 0.0f) speed = 0.0f;

    // Reset forces and torques for this frame
    totalForce = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
    totalYawTorque = 0.0f;
    glm::vec4 longitudinalForce = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);

    // --- 1. Calculate Longitudinal Forces ---

    // Rolling Resistance (Frr = -Crr * v)
    glm::vec4 F_rr = -C_rr * velocity;
    totalForce += F_rr;

    // Aerodynamic Drag (Fdrag = -C_drag * v * |v|)
    glm::vec4 F_drag = -C_drag * velocity * speed;
    totalForce += F_drag;

    // Braking Force
    float F_braking_magnitude = brakeInput * mass * GRAVITY * friction_coefficient; // Simple constant brake force
    if (speed > 0.01f && brakeInput > 0.0f) { // Only apply if moving forward and braking
        glm::vec4 F_braking = -carHeading * F_braking_magnitude;
        longitudinalForce += F_braking;
        currentBrakingForce = glm::length(F_braking);
    } else if (speed < 0.01f && brakeInput > 0.0f) {
        // If stopped and braking, ensure velocity becomes zero
        velocity = glm::vec4(0.0f);
        wheelAngularVelocity = 0.0f;
        currentBrakingForce = 0.0f;
    } else {
        currentBrakingForce = 0.0f;
    }

    // Engine Torque and Drive Force
    float currentGearRatio = 0.0f;
    if (!gearRatios.empty()) {
        // Simplified gear selection: Always use 1st gear for now, or implement auto/manual gear shifting
        currentGearRatio = gearRatios[0]; // Example: always use first gear
    }

    // Calculate current Engine RPM from wheel angular velocity
    // rpm = wheel_rot_rate * gear_ratio * differential_ratio * (60 / (2 * pi))
    currentEngineRPM = wheelAngularVelocity * currentGearRatio * differentialRatio * (60.0f / (2.0f * M_PI));
    if (wheelAngularVelocity == 0.0f && throttleInput > 0.0f) {
        // When starting from standstill, assume min RPM to get moving
        currentEngineRPM = engineIdleRPM;
    }

    float maxEngineTorque = lookupTorque(currentEngineRPM);
    float T_engine = throttleInput * maxEngineTorque; // Actual engine torque

    float T_drive = T_engine * currentGearRatio * differentialRatio * transmissionEfficiency;
    glm::vec4 F_drive = carHeading * (T_drive / wheelRadius); // Drive force from engine

    // --- 2. Longtitudinal Tire Forces & Slip Ratio ---

    // Calculate slip ratio for drive wheels
    // This assumes rear-wheel drive.
    float Rw_omega = wheelAngularVelocity * wheelRadius;
    float slipRatio = 0.0f;
    if (v_long != 0.0f) {
        slipRatio = (Rw_omega - v_long) / std::abs(v_long);
    } else if (Rw_omega != 0.0f) { // If car is stopped but wheels are spinning
        slipRatio = (Rw_omega > 0.0f) ? 1.0f : -1.0f; // Can be > 1 or < -1 for significant slip
    }
    currentSlipRatio = slipRatio; // For debugging

    // Max Longitudinal Traction (simplified Fmax = mu * W) considering weight transfer
    float W_rear = (cgToFrontAxle / wheelbase) * (mass * GRAVITY) + (cgHeight / wheelbase) * mass * acceleration.x; // a.x is longitudinal acceleration
    float maxTractionForceRear = friction_coefficient * W_rear;
    if (maxTractionForceRear < 0) maxTractionForceRear = 0; // Prevent negative load

    // Longtitudinal force from traction (due to slip)
    // Simplified linear model: Flong = C_traction * slip_ratio, capped by Fmax
    float F_traction_magnitude = C_traction * slipRatio;
    F_traction_magnitude = std::min(std::abs(F_traction_magnitude), maxTractionForceRear) * glm::sign(slipRatio);

    glm::vec4 F_traction = carHeading * F_traction_magnitude;
    longitudinalForce += F_traction;
    currentLongitudinalForce = F_traction_magnitude; // For debugging

    // Sum all longitudinal forces (includes drive, resistance, braking, traction due to slip)
    totalForce += longitudinalForce;


    // --- 3. Calculate Lateral Forces (for turning) ---

    // Sideslip angle (beta) of the car body
    float sideslipAngle = 0.0f;
    if (v_long != 0.0f) {
        sideslipAngle = std::atan2(v_lat, v_long);
    }

    // Front wheel slip angle
    float slipAngleFront = sideslipAngle + steerAngle - (yawRate * cgToFrontAxle / v_long);
    // Rear wheel slip angle
    float slipAngleRear = sideslipAngle + (yawRate * cgToRearAxle / v_long);

    // Handle division by zero if v_long is very small, use a small epsilon or linear approximation
    if (std::abs(v_long) < 0.1f) { // Arbitrary small speed threshold
        slipAngleFront = sideslipAngle + steerAngle - (yawRate * cgToFrontAxle / 0.1f);
        slipAngleRear = sideslipAngle + (yawRate * cgToRearAxle / 0.1f);
        // At very low speeds, slip angle is almost proportional to lateral velocity / yaw rate
        // More complex handling for low speed physics might be needed.
    }

    currentSlipAngleFront = slipAngleFront; // For debugging
    currentSlipAngleRear = slipAngleRear;   // For debugging

    // Weight on front and rear axles (including dynamic weight transfer)
    float a_long = glm::dot(acceleration, carHeading); // Longitudinal acceleration
    float W_front = (cgToRearAxle / wheelbase) * (mass * GRAVITY) - (cgHeight / wheelbase) * mass * a_long;
    float W_rear_dynamic = (cgToFrontAxle / wheelbase) * (mass * GRAVITY) + (cgHeight / wheelbase) * mass * a_long;

    W_front = std::max(0.0f, W_front); // Ensure non-negative weight
    W_rear_dynamic = std::max(0.0f, W_rear_dynamic); // Ensure non-negative weight


    // Lateral force for front wheels (total for both)
    // Flateral = C_cornering_stiffness * alpha, capped by Fmax = mu * W
    float F_lateral_front_magnitude = C_cornering_stiffness * slipAngleFront;
    float maxLateralForceFront = friction_coefficient * W_front;
    F_lateral_front_magnitude = std::min(std::abs(F_lateral_front_magnitude), maxLateralForceFront) * glm::sign(slipAngleFront);

    // Lateral force for rear wheels (total for both)
    float F_lateral_rear_magnitude = C_cornering_stiffness * slipAngleRear;
    float maxLateralForceRear = friction_coefficient * W_rear_dynamic;
    F_lateral_rear_magnitude = std::min(std::abs(F_lateral_rear_magnitude), maxLateralForceRear) * glm::sign(slipAngleRear);

    currentLateralForceFront = F_lateral_front_magnitude; // For debugging
    currentLateralForceRear = F_lateral_rear_magnitude;   // For debugging


    // Convert lateral forces to world space and apply to totalForce
    // Front lateral force is applied perpendicular to steered wheel direction
    glm::vec4 steeredFrontHeading = glm::rotate(carHeading, steerAngle, glm::vec3(0.0f, 1.0f, 0.0f)); // Rotate around Y-axis
    glm::vec4 steeredFrontSide = glm::rotate(carSide, steerAngle, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::vec4 F_lateral_front = steeredFrontSide * (-F_lateral_front_magnitude); // Negative because force opposes slip

    glm::vec4 F_lateral_rear = carSide * (-F_lateral_rear_magnitude); // Rear wheels always aligned with car body

    totalForce += F_lateral_front;
    totalForce += F_lateral_rear;

    // --- 4. Calculate Yaw Torque ---
    // Torque = Force * perpendicular_distance
    // Front wheels contribute torque: F_lateral_front_magnitude * cos(steerAngle) * cgToFrontAxle
    // Rear wheels contribute torque: -F_lateral_rear_magnitude * cgToRearAxle
    // The sign depends on your coordinate system and how torque is defined (clockwise/counter-clockwise)
    // Assume positive torque turns car counter-clockwise (increasing yawAngle)
    totalYawTorque += F_lateral_front_magnitude * cos(steerAngle) * cgToFrontAxle;
    totalYawTorque += F_lateral_rear_magnitude * (-cgToRearAxle);


    // --- 5. Update Wheel Angular Velocity ---
    // Total torque on drive axle = Drive Torque - Traction Torque - Brake Torque
    // This is complex as it requires knowing the friction at the contact patch *before* acceleration.
    // The tutorial introduces it then goes back to it, and says F_drive = T_drive / Rw
    // and traction torque = traction force * wheel radius. These are opposing.
    // Let's simplify and use the previous F_traction_magnitude for traction torque
    float T_traction_rear = F_traction_magnitude * wheelRadius; // Torque resisting drive

    float T_brake_axle = brakeInput * 1000.0f; // Example: 1000 N.m max brake torque

    // Net torque on the drive axle
    float netWheelTorque = T_drive - T_traction_rear;
    if (brakeInput > 0.0f) {
        netWheelTorque -= T_brake_axle * glm::sign(wheelAngularVelocity); // Apply brake torque opposite to rotation
    }

    if (netWheelTorque > maxTractionForceRear * wheelRadius) { // Cap drive torque to prevent excessive wheel spin
        netWheelTorque = maxTractionForceRear * wheelRadius;
    }
    if (netWheelTorque < -maxTractionForceRear * wheelRadius) { // Cap braking torque
        netWheelTorque = -maxTractionForceRear * wheelRadius;
    }


    float wheelAngularAcceleration = 0.0f;
    if (wheelInertia > 0.0f) {
        wheelAngularAcceleration = netWheelTorque / wheelInertia;
    }
    wheelAngularVelocity += wheelAngularAcceleration * deltaTime;

    // Prevent reverse wheel spin if speed is zero and not accelerating backwards
    if (speed < 0.01f && v_long == 0.0f && netWheelTorque < 0.0f) {
        wheelAngularVelocity = std::max(0.0f, wheelAngularVelocity); // Prevent negative wheel speed if car is stopped
    }
    if (speed < 0.01f && v_long == 0.0f && netWheelTorque > 0.0f) {
         // Allow wheel spin from standstill but cap it to avoid infinite acceleration
        wheelAngularVelocity = std::min(wheelAngularVelocity, static_cast<float>((engineMaxRPM / 60.0f) * (2.0f * M_PI) / (gearRatios[0] * differentialRatio))
); }


    // --- 6. Integrate Linear and Angular Motion ---

    // Linear Acceleration (a = F / M)
    acceleration = totalForce / mass;
    acceleration.w = 0.0f; // Ensure w is 0 for acceleration vector

    // Linear Velocity (v = v + dt * a)
    velocity += acceleration * deltaTime;

    // Linear Position (p = p + dt * v)
    position += velocity * deltaTime;

    // Yaw Acceleration (angular_accel = torque / inertia)
    yawAcceleration = totalYawTorque / inertia;

    // Yaw Rate (angular_velocity = angular_velocity + dt * angular_accel)
    yawRate += yawAcceleration * deltaTime;

    // Yaw Angle (angle = angle + dt * angular_velocity)
    yawAngle += yawRate * deltaTime;

    // Normalize yaw angle to keep it within [0, 2*PI) or (-PI, PI]
    yawAngle = fmod(yawAngle, 2.0f * M_PI);
    if (yawAngle < 0.0f) {
        yawAngle += 2.0f * M_PI;
    }

    // A final check to stop the car completely if speed is very low and no throttle/brake
    if (speed < 0.1f && throttleInput == 0.0f && brakeInput == 0.0f) {
        velocity = glm::vec4(0.0f);
        acceleration = glm::vec4(0.0f);
        yawRate = 0.0f;
        yawAcceleration = 0.0f;
        wheelAngularVelocity = 0.0f;
    }
}
