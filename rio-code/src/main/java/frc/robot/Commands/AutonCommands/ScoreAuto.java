package frc.robot.Commands.AutonCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.manipulator.SetManipulatorWheelSpeed;
import frc.robot.subsystems.RobotMode.DriveMode;

public class ScoreAuto extends SequentialCommandGroup{
    
    public ScoreAuto() {
        addCommands(
            new SetManipulatorWheelSpeed(0.5),
            new WaitUntilCommand(() -> !TriggerBoard.isCoralInManipulator()),
            Robot.drivetrain.runOnce(() -> {
                Robot.robotMode.setDriveMode(DriveMode.Brake);
                Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-0.25));
            }),
            new WaitCommand(0.35),
            Robot.drivetrain.runOnce(() -> {
                Robot.robotMode.setDriveMode(DriveMode.Auton);
                Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0));
            })
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);

    }

}
