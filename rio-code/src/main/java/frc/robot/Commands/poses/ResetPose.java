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
            new SetIntakePosition(0),
            new SetIntakeWheelSpeed(-0.5),
            new SetArmAngle(-0.3),
            new SetElevatorPosition(0.5),
            new SetArmAngle(-0.128),
            new SetIntakePosition(Constants.Intake.pivotStowPosition),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(RobotMode.transit);}).ignoringDisable(true)
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
