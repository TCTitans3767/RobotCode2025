package frc.robot.utils;

import dev.doglog.DogLog;

public class Logger extends DogLog{
    public static void logLimitError(String message) {
        DogLog.log("Error/Limit", message);
    }
    
    public static void logElevatorSetHeight(double height) {
        DogLog.log("Elevator/Set Height", height);
    }

    public static void logElevatorHeight(double height) {
        DogLog.log("Elevator/Height", height);
    }

    public static void logElevatorMotorSpeed(double speed) {
        DogLog.log("Elevator/Motor Speed", speed);
    }

    public static void logElevatorMotorVelocity(double velocity) {
        DogLog.log("Elevator/Motor Velocity", velocity);
    }

    public static void logElevatorAtPosition(boolean isAtPosition) {
        DogLog.log("Elevator/Is At Position", isAtPosition);
        
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
