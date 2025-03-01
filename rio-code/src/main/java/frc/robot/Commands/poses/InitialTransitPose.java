package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.Intake.SetIntakeWheelSpeed;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.RobotMode;

public class InitialTransitPose extends SequentialCommandGroup{
    
    public InitialTransitPose() {

        addCommands(
            new SetArmAngle(-0.3),
            new SetIntakeWheelSpeed(0),
            new SetIntakePosition(0.32),
            new SetElevatorPosition(0.5),
            new SetArmAngle(-0.128),
            new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive); Robot.manipulator.setSpeed(0); Robot.limelight.turnOnAprilTags();}),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(RobotMode.transit);}).ignoringDisable(true)
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
