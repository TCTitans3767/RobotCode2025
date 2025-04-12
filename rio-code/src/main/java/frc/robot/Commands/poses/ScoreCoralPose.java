package frc.robot.Commands.poses;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ButtonBox;
import frc.robot.Constants;
import frc.robot.DashboardButtonBox;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;

public class ScoreCoralPose extends SequentialCommandGroup{

    private class scoreAndTransit extends SequentialCommandGroup {
        public scoreAndTransit() {
            addCommands(
                new InstantCommand(() -> {
                    if (TriggerBoard.isL1Selected() && !DriverStation.isAutonomousEnabled()) {
                        Robot.elevator.setPosition(0.02);
                        Robot.robotMode.setDriveMode(DriveMode.Brake);
                        Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(1.8));
                    } else {
                        Robot.robotMode.setDriveMode(DriveMode.Brake);
                        Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-1.5));
                    }

                }),
                new WaitCommand(0.25),
                new InstantCommand(() -> {if (!DriverStation.isAutonomousEnabled()) {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);} else {Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0));}}),
                new InstantCommand(() -> {
                    Robot.manipulator.setSpeed(0);
                    Robot.robotMode.setCurrentMode(RobotMode.coralFloorPose);
                })
            );
        }
    }

    private double timer = 0;

    // private Map<String, Command> commandMap = new HashMap<String, Command>();

    public ScoreCoralPose() {

        // commandMap.put("4", new SetArmAngle(Constants.L4Measurements.armAngle));
        // commandMap.put("3", new SetArmAngle(Constants.L3Measurements.armAngle));
        // commandMap.put("2", new SetArmAngle(Constants.L2Measurements.armAngle));

        addCommands(
            // new SelectCommand<String>(commandMap, DashboardButtonBox::getSelectedLevelString),
            new InstantCommand(() -> {
                if (TriggerBoard.isL1Selected()) {
                    Robot.intake.scoreL1();
                } else {
                    Robot.manipulator.setSpeed(0.4);
                }
            }),
            new WaitCommand(0.15),
            new ConditionalCommand(
                new ParallelCommandGroup(
                    new SetArmAngle(0.17),
                    new SetElevatorPosition(1.1)
                ),
                Commands.none(),
                TriggerBoard::isL4Selected
            ), 
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
