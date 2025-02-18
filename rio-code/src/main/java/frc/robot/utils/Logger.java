package frc.robot.utils;

import dev.doglog.DogLog;

public class Logger extends DogLog {
    public static void logLimitError(String message) {
        DogLog.log("Error/Limit", message);
    }

    public static void logArmSetPosition(double rotations) {
        DogLog.log("Arm/Set Position", rotations);
    }

    public static void logArmAbsolute(double degrees) {
        DogLog.log("Arm/Absolute Degrees", degrees);
    }

    public static void logArmMotorPosition(double position) {
        DogLog.log("Arm/Motor Position", position);
    }

    public static void logArmMotorSpeed(double speed) {
        DogLog.log("Arm/Motor Speed", speed);
    }

    public static void logArmMotorVelocity(double velocity) {
        DogLog.log("Arm/Motor Velocity", velocity);
    }

    public static void logArmAtPosition(boolean isAtPosition) {
        DogLog.log("Arm/at position", isAtPosition);
    }
}
