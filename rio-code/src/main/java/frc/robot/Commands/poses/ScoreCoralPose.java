package frc.robot.Commands.poses;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ButtonBox;
import frc.robot.DashboardButtonBox;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;

public class ScoreCoralPose extends SequentialCommandGroup{

    private class scoreAndTransit extends SequentialCommandGroup {
        public scoreAndTransit() {
            addCommands(
                new InstantCommand(() -> {
                    if (DashboardButtonBox.getSelectedLevelString() == "1") {
                        Robot.robotMode.setDriveMode(DriveMode.Brake);
                        Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.25));
                    } else {
                        Robot.robotMode.setDriveMode(DriveMode.Brake);
                        Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-0.25));
                    }

                }),
                new WaitCommand(0.2),
                new InstantCommand(() -> Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive)),
                new InstantCommand(() -> {
                    Robot.manipulator.setSpeed(0);
                    Robot.robotMode.setCurrentMode(RobotMode.coralFloorPose);
                })
            );
        }
    }

    private double timer = 0;

    public ScoreCoralPose() {

        addCommands(
            new InstantCommand(() -> {
                if (DashboardButtonBox.getSelectedLevelString() == "1") {
                    Robot.manipulator.setSpeed(0.03);
                } else {
                    Robot.manipulator.setSpeed(0.25);
                }
            }),
            new WaitCommand(0.1),
            new ConditionalCommand(
                new InstantCommand(() -> {
                    Robot.manipulator.setSpeed(0);
                    Robot.robotMode.setCurrentMode(RobotMode.knockOffAlgaePose);
                }),
                new scoreAndTransit(),
                DashboardButtonBox::isAlgaeKnockoffOn
            )
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
