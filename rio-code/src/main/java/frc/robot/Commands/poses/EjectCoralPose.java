package frc.robot.Commands.poses;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.DashboardButtonBox;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;

public class EjectCoralPose extends SequentialCommandGroup{

    private double timer = 0;

    public EjectCoralPose() {

        addCommands(
            new InstantCommand(() -> {
                if (DashboardButtonBox.getSelectedLevelString() == "1") {
                    Robot.manipulator.setSpeed(0.05);
                } else {
                    Robot.manipulator.setSpeed(0.25);
                }
            }),
            new WaitCommand(0.4),
            new InstantCommand(() -> {
                Robot.robotMode.setDriveMode(DriveMode.Brake);
                Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-0.25));
            }),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
                Robot.manipulator.setSpeed(0);
                Robot.robotMode.setCurrentMode(RobotMode.transitPose);
            }
        ));

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
