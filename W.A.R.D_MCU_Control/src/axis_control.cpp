#include "axis_control.h"

AxisControl::AxisControl(uint8_t stepPin, uint8_t dirPin, bool enableAs5600)
    : m_stepper(AccelStepper::DRIVER, stepPin, dirPin),
      m_as5600Enabled(enableAs5600) {}

void AxisControl::begin(float as5600Offset) {
    m_as5600Offset = as5600Offset;
    if (m_as5600Enabled) {
        if (!m_as5600.begin()) {
            Serial.println("AS5600 not found.");
        } else {
            m_as5600Available = true;
            Serial.println("AS5600 ready.");
        }
    }
}

void AxisControl::configure(float maxSpeed, int16_t acceleration, long startPosition) {
    setMaxSpeed(maxSpeed);
    setAcceleration(acceleration);
    setCurrentPosition(startPosition);
}

void AxisControl::setMaxSpeed(float speed) {
    m_stepper.setMaxSpeed(speed);
}

void AxisControl::setSpeed(int16_t speed) {
    m_speed = speed;
    m_stepper.setSpeed(static_cast<float>(m_speed));
}

void AxisControl::setSpeedMode(bool enabled) {
    m_speedMode = enabled;
}

bool AxisControl::isSpeedMode() const {
    return m_speedMode;
}

void AxisControl::setAcceleration(int16_t acceleration) {
    if (acceleration < 0) {
        acceleration = static_cast<int16_t>(-acceleration);
    }
    m_acceleration = acceleration;
    m_stepper.setAcceleration(static_cast<float>(m_acceleration));
}

void AxisControl::move(long delta) {
    m_stepper.move(delta);
}

void AxisControl::moveTo(long position) {
    m_stepper.moveTo(position);
}

void AxisControl::stop() {
    m_stepper.stop();
}

void AxisControl::holdPosition() {
    m_stepper.moveTo(m_stepper.currentPosition());
}

void AxisControl::stopAndHold() {
    stop();
    holdPosition();
    setSpeed(0);
}

void AxisControl::startHomingToZero() {
    m_homingToZero = true;
}

bool AxisControl::isHoming() const {
    return m_homingToZero;
}

void AxisControl::update(uint32_t nowMs) {
    if (m_as5600Available && (nowMs - m_lastAs5600ReadMs >= kAs5600ReadIntervalMs)) {
        m_lastAs5600ReadMs = nowMs;
        if (!m_as5600.isMagnetDetected()) {
            m_as5600AngleValid = false;
            Serial.println("AS5600 magnet not detected");
        } else {
            const float rawAngle = (m_as5600.getRawAngle() * 360.0f) / 4096.0f;
            float angle = -(rawAngle - m_as5600Offset);
            if (angle > 180.0f) {
                angle -= 360.0f;
            } else if (angle <= -180.0f) {
                angle += 360.0f;
            }
            m_as5600Angle = angle;
            m_as5600AngleValid = true;
            Serial.print("AS5600 angle: ");
            Serial.println(angle, 2);
        }
    }

    if (m_homingToZero && m_as5600AngleValid) {
        Serial.println("Homing X to zero...");
        const float absAngle = (m_as5600Angle < 0.0f) ? -m_as5600Angle : m_as5600Angle;
        if (absAngle <= HOME_TOLERANCE_DEG) {
            m_homingToZero = false;
            m_stepper.setSpeed(0.0f);
        } else {
            const float direction = (m_as5600Angle > 0.0f) ? -1.0f : 1.0f;
            m_stepper.setSpeed(direction * HOME_SPEED);
        }
        m_stepper.runSpeed();
        return;
    }

    if (m_speedMode) {
        m_stepper.setSpeed(static_cast<float>(m_speed));
        m_stepper.runSpeed();
    } else {
        m_stepper.run();
    }
}

void AxisControl::setCurrentPosition(long position) {
    m_stepper.setCurrentPosition(position);
}

long AxisControl::currentPosition() {
    return m_stepper.currentPosition();
}
