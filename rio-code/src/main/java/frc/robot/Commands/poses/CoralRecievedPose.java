package frc.robot.Commands.poses;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;

public class CoralRecievedPose extends SequentialCommandGroup{
    
    public CoralRecievedPose() {

        addCommands(
                new InstantCommand(() -> {
                    Robot.robotMode.setDriveMode(DriveMode.Brake);
                    Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.35));
                }),
                new WaitCommand(0.25),
                new InstantCommand(() -> {if (!DriverStation.isAutonomousEnabled()) {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);} else {Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0));}}),
                new InstantCommand(() -> {
                    Robot.manipulator.setSpeed(0);
                    Robot.robotMode.setCurrentMode(RobotMode.transitPose);
                })
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
