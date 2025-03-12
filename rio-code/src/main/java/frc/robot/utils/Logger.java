package frc.robot.utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;

public class Logger extends DogLog{
    public static void logSystemError(String message) {
        DogLog.log("SystemError", message);
        System.err.println("SystemError:" + message);
    }

    public static void logButtonBox(String message) {
        DogLog.log("ButtonBox", message);
    }

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

    public static void logElevatorMotorCurrent(double current) {
        DogLog.log("Elevator/Motor Current", current);
    }

    public static void logElevatorAtPosition(boolean isAtPosition) {
        DogLog.log("Elevator/Is At Position", isAtPosition);
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

    public static void logCameraTagFilter(String cameraName, int[] filter) {
        DogLog.log(cameraName + "/Tag Filter", filter);
    }

    public static void logCameraEstimatedPose(String cameraName, Pose2d estimatedPose) {
        DogLog.log(cameraName + "/Estimated Pose X", estimatedPose.getX());
        DogLog.log(cameraName + "/Estimated Pose Y", estimatedPose.getY());
        DogLog.log(cameraName + "/Estimated Pose Rotation", estimatedPose.getRotation().getDegrees());
    }

    public static void logCameraTagsVisible(String cameraName, int[] visibleTags) {
        DogLog.log(cameraName + "/Visible Tags", visibleTags);
    }

    public static void logCameraEstimationEnabled(String cameraName, boolean estimationEnabled) {
        DogLog.log(cameraName + "/Estimation Enabled", estimationEnabled);
    }
    
}
