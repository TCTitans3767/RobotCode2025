package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode;

public class DeployClimberPose extends SequentialCommandGroup{
    
    public DeployClimberPose() {

        addCommands(
            new InstantCommand(() -> Robot.climber.setSpeed(0.6)),
            new WaitCommand(0.5),
            new InstantCommand(() -> Robot.robotMode.setDriveModeCommand(RobotMode.climbControllerDrive)),
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(RobotMode.finalClimb))
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }
    
}
