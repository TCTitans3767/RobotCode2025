package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.RobotMode;

public class ResetPose extends SequentialCommandGroup{
    
    public ResetPose() {

        addCommands(
            // new SetIntakePosition(0),
            new SetArmAngle(-0.3),
            new SetElevatorPosition(0.5),
            new SetArmAngle(0.128),
            new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);}).ignoringDisable(true),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(RobotMode.transit);}).ignoringDisable(true)
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
