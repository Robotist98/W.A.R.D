#include "axis_control.h"

AxisControl::AxisControl(uint8_t stepPin, uint8_t dirPin, bool enableAs5600)
    : m_stepper(AccelStepper::DRIVER, stepPin, dirPin),
      m_as5600Enabled(enableAs5600) {}

bool AxisControl::begin(float as5600Offset) {
    m_as5600Offset = as5600Offset;
    if (m_as5600Enabled) {
        if (!m_as5600.begin()) {
            m_as5600Available = false;
            return false;
        } else {
            m_as5600Available = true;
        }
    }
    return true;
}

void AxisControl::configure(float positionMaxSpeed, int16_t acceleration, long startPosition) {
    setPositionMaxSpeed(positionMaxSpeed);
    setAcceleration(acceleration);
    setCurrentPosition(startPosition);
}

void AxisControl::setPositionMaxSpeed(float speed) {
    m_stepper.setMaxSpeed(speed);
}

void AxisControl::setSpeedModeSpeed(int16_t speed) {
    m_speed = speed;
    m_stepper.setSpeed(static_cast<float>(m_speed));
}

int16_t AxisControl::speedModeSpeed() const {
    return m_speed;
}

void AxisControl::setMode(Mode mode) {
    m_mode = mode;
}

AxisControl::Mode AxisControl::mode() const {
    return m_mode;
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
    setSpeedModeSpeed(0);
}

void AxisControl::update(uint32_t nowMs) {
    (void)nowMs;
    readAs5600();
    if (m_mode == Mode::Homing && updateHoming()) {
        return;
    }
    runMotion();
}

bool AxisControl::startHomingToZero() {
    if (!m_as5600Available) {
        return false;
    }
    m_mode = Mode::Homing;
    return true;
}

bool AxisControl::isHoming() const {
    return m_mode == Mode::Homing;
}

void AxisControl::setCurrentPosition(long position) {
    m_stepper.setCurrentPosition(position);
}

long AxisControl::currentPosition() {
    return m_stepper.currentPosition();
}

float AxisControl::readAs5600() {
    const uint32_t nowMs = millis();
    if (!m_as5600Available || (nowMs - m_lastAs5600ReadMs < kAs5600ReadIntervalMs)) {
        return NAN;
    }
    m_lastAs5600ReadMs = nowMs;
    if (!m_as5600.isMagnetDetected()) {
        m_as5600AngleValid = false;
        return NAN;
    }

    const float rawAngle = (m_as5600.getRawAngle() * 360.0f) / 4096.0f;
    float angle = -(rawAngle - m_as5600Offset);
    if (angle > 180.0f) {
        angle -= 360.0f;
    } else if (angle <= -180.0f) {
        angle += 360.0f;
    }
    m_as5600Angle = angle;
    m_as5600AngleValid = true;
    return m_as5600Angle;
}

bool AxisControl::updateHoming() {
    if (!m_as5600AngleValid) {
        return false;
    }
    const float absAngle = (m_as5600Angle < 0.0f) ? -m_as5600Angle : m_as5600Angle;
    if (absAngle <= HOME_TOLERANCE_DEG) {
        m_stepper.setSpeed(0.0f);
        return false;
    } else {
        const float direction = (m_as5600Angle > 0.0f) ? -1.0f : 1.0f;
        m_stepper.setSpeed(direction * HOME_SPEED);
    }
    m_stepper.runSpeed();
    return true;
}

void AxisControl::runMotion() {
    switch (m_mode) {
        case Mode::Speed:
            m_stepper.setSpeed(static_cast<float>(m_speed));
            m_stepper.runSpeed();
            break;
        case Mode::Position:
            m_stepper.run();
            break;
        case Mode::Idle:
            break;
        case Mode::Homing:
            m_stepper.setSpeed(static_cast<float>(m_speed));
            m_stepper.runSpeed();
            break;
    }
}
