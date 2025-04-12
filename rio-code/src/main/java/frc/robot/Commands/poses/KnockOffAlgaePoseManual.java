package frc.robot.Commands.poses;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;

public class KnockOffAlgaePoseManual extends SequentialCommandGroup{
    
    public KnockOffAlgaePoseManual() {

        addCommands(
            new ParallelCommandGroup(
                new SetArmAngle(-0.02),
                new SetElevatorPosition(1.05)
            ),
            new WaitUntilCommand(() -> Robot.joystick.a().getAsBoolean()),
            new InstantCommand(() -> Robot.elevator.setSpeed(-0.4)),
            new WaitCommand(0.2),
            new ParallelRaceGroup(
                new WaitUntilCommand(KnockOffAlgaePose::isElevatorAtPosition),
                new WaitUntilCommand(KnockOffAlgaePose::isManipulatorTouchingAlgae)
            ),
            new InstantCommand(() -> {
                Robot.elevator.setSpeed(0);
                Robot.robotMode.setDriveMode(DriveMode.Brake);
                Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-0.5));
            }),
            new WaitCommand(0.6),
            new InstantCommand(() -> {Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0)); Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);}),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(RobotMode.coralFloorPose); Robot.manipulator.setSpeed(0);})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    public static boolean isAlignCommandFinsihed() {
        return RobotMode.alignWithAlgae.isFinished();
    }

    public static boolean isElevatorAtPosition() {
        return Robot.elevator.getPosition() <= 0.05;
    }

    public static boolean isManipulatorTouchingAlgae() {
        return (Robot.arm.getMotionMagicError() > 0.01 || Robot.arm.getMotionMagicError() < -0.01) && Robot.elevator.getPosition() <= 0.85;
    }

}
