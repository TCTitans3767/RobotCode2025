#pragma once

#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <rev/SparkMax.h>

class Drivetrain {
public:
    Drivetrain();
    void Drive(double leftSpeed, double rightSpeed);
    void ArcadeDrive(double speed, double rotation);
    void Stop();
    
private:
    // Define Spark MAX controllers (CHANGE IDs TO MATCH YOUR BOT)
    rev::spark::SparkMax m_leftLeader{3, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax m_leftFollower{5, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax m_rightLeader{2, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax m_rightFollower{4, rev::spark::SparkLowLevel::MotorType::kBrushless};
};
