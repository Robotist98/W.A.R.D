#ifndef AXIS_CONTROL_H
#define AXIS_CONTROL_H

#include <Arduino.h>
#include "AccelStepper.h"
#include <Adafruit_AS5600.h>
#include "config.h"
#include <Wire.h>

class AxisControl {
public:
    enum class Mode {
        Position,
        Speed,
        Homing,
        Idle
    };
    // Create a step/dir axis; optionally enable AS5600 sensor + homing support.
    AxisControl(uint8_t stepPin, uint8_t dirPin, bool enableAs5600 = false);

    // Initialize hardware; pass sensor offset in degrees if AS5600 enabled.
    void begin(float as5600Offset = 0.0f);
    // Configure position-mode max speed, acceleration, and optional start position.
    void configure(float positionMaxSpeed, int16_t acceleration, long startPosition = 0);
    // Configure position-mode max speed (steps/sec).
    void setPositionMaxSpeed(float speed);
    // Set target speed for speed mode (steps/sec).
    void setSpeedModeSpeed(int16_t speed);
    // Set how update() drives the axis.
    void setMode(Mode mode);
    // Current update() mode.
    Mode mode() const;
    // Set acceleration (steps/sec^2).
    void setAcceleration(int16_t acceleration);
    // Move relative by delta steps.
    void move(long delta);
    // Move to absolute position in steps.
    void moveTo(long position);
    // Request a stop (AccelStepper decelerates).
    void stop();
    // Hold current position by setting target to current.
    void holdPosition();
    // Stop, hold position, and zero the speed target.
    void stopAndHold();
    // Update sensor + motion; handles homing if active.
    void update(uint32_t nowMs);
    // Begin homing to zero using AS5600 angle.
    void startHomingToZero();
    // True while homing is active.
    bool isHoming() const;
    // Set the stepper position without motion.
    void setCurrentPosition(long position);
    // Current stepper position.
    long currentPosition();
private:
    AccelStepper m_stepper;
    int16_t m_speed = 0;
    int16_t m_acceleration = 0;

    Adafruit_AS5600 m_as5600;
    bool m_as5600Enabled = false;
    bool m_as5600Available = false;
    float m_as5600Offset = 0.0f;
    uint32_t m_lastAs5600ReadMs = 0;
    float m_as5600Angle = 0.0f;
    bool m_as5600AngleValid = false;
    Mode m_mode = Mode::Position;
    static constexpr uint32_t kAs5600ReadIntervalMs = 100;


};






#endif
