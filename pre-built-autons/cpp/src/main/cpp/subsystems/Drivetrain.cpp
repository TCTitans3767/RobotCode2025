#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() {

}

void Drivetrain::Drive(double leftSpeed, double rightSpeed) {
    m_leftLeader.Set(-leftSpeed);
    m_leftFollower.Set(m_leftLeader.Get());

    m_rightLeader.Set(rightSpeed);
    m_rightFollower.Set(m_rightLeader.Get());
}

void Drivetrain::Stop() {
    m_leftLeader.Set(0);
    m_leftFollower.Set(0);
    m_rightLeader.Set(0);
    m_rightFollower.Set(0);
}
