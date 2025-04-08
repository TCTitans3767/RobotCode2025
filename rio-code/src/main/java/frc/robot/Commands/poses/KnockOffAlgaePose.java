package frc.robot.Commands.poses;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

public class KnockOffAlgaePose extends SequentialCommandGroup{

    Command algaeAlign = new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.alignWithAlgae);});

    Command bounceToTransit = new SequentialCommandGroup(
        new InstantCommand(() -> {
            if (TriggerBoard.isL1Selected()) {
                Robot.robotMode.setDriveMode(DriveMode.Brake);
                Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.55));
            } else {
                Robot.robotMode.setDriveMode(DriveMode.Brake);
                Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-0.35));
            }
        }),
        new WaitCommand(0.25),
        new InstantCommand(() -> {if (!DriverStation.isAutonomousEnabled()) {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);} else {Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0));}}),
        new InstantCommand(() -> {
            Robot.manipulator.setSpeed(0);
            Robot.robotMode.setCurrentMode(RobotMode.coralFloorPose);
        })
    );
    
    public KnockOffAlgaePose() {

        addCommands(
            new ConditionalCommand(algaeAlign, new InstantCommand(() -> Robot.robotMode.setCurrentMode(RobotMode.transitPose)), TriggerBoard::isL4Selected),
            new WaitUntilCommand((KnockOffAlgaePose::isAlignCommandFinsihed)).withTimeout(1),
            new ParallelCommandGroup(
                new SetArmAngle(-0.07),
                new SetElevatorPosition(1.1)
            ),
            new ParallelRaceGroup(
                new RunCommand(() -> Robot.manipulator.setSpeed(-0.7)),
                new RunCommand(() -> Robot.elevator.setSpeed(-0.4)),
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
        System.out.println("align finished: " + RobotMode.alignWithAlgae.isFinished());
        return RobotMode.alignWithAlgae.isFinished() || TriggerBoard.isCoralButtonPressed();
    }

    public static boolean isElevatorAtPosition() {
        return Robot.elevator.getPosition() <= 0.05;
    }

    public static boolean isManipulatorTouchingAlgae() {
        return (Robot.arm.getMotionMagicError() > 0.01 || Robot.arm.getMotionMagicError() < -0.01) && Robot.elevator.getPosition() <= 0.85;
    }

}
