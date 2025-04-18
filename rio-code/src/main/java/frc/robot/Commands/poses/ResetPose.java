package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.Intake.SetIntakeWheelSpeed;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.Commands.manipulator.SetManipulatorWheelSpeed;
import frc.robot.subsystems.RobotMode;

public class ResetPose extends SequentialCommandGroup{
    
    public ResetPose() {

        addCommands(
            new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);}).ignoringDisable(true),
            new SetManipulatorWheelSpeed(0),
            new SetIntakePosition(0).withTimeout(3),
            new SetIntakeWheelSpeed(-60),
            new SetArmAngle(0.05),
            new SetElevatorPosition(0.5),
            new SetArmAngle(-0.122),
            new SetIntakePosition(Constants.Intake.pivotStowPosition).withTimeout(3),
            new InstantCommand(() -> {Robot.intake.resetWheelSpeed(); Robot.robotMode.setCurrentMode(RobotMode.transit);}).ignoringDisable(true)
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
