package frc.robot.Commands.poses;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode;

public class CoralRecievedPose extends SequentialCommandGroup{
    
    public CoralRecievedPose() {

        addCommands(
            new ParallelRaceGroup(
                new RunCommand(() -> {Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.25));}, Robot.drivetrain),
                new RunCommand(() -> {Robot.manipulator.setSpeed(-0.05);}, Robot.manipulator),
                new WaitCommand(0.15)
            ),
            new InstantCommand(() -> {
                Robot.manipulator.setSpeed(0);
                Robot.robotMode.setCurrentMode(RobotMode.transitPose);
            })
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
