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
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;

public class KnockOffAlgaePose extends SequentialCommandGroup{

    Command algaeAlign = new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.alignWithAlgae);});
    
    public KnockOffAlgaePose() {

        addCommands(
            algaeAlign,
            new WaitUntilCommand(KnockOffAlgaePose::isAlignCommandFinsihed),
            new ParallelCommandGroup(
                new SetArmAngle(-0.35),
                new SetElevatorPosition(1.1)
            ),
            new ParallelRaceGroup(
                new RunCommand(() -> Robot.manipulator.setSpeed(-0.7)),
                new RunCommand(() -> Robot.elevator.setSpeed(-0.2)),
                new WaitUntilCommand(KnockOffAlgaePose::isElevatorAtPosition),
                new WaitUntilCommand(KnockOffAlgaePose::isManipulatorTouchingAlgae)
            ),
            new InstantCommand(() -> {
                Robot.elevator.setSpeed(0);
                Robot.robotMode.setDriveMode(DriveMode.Brake);
                Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-0.15));
            }),
            new WaitCommand(0.5),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(RobotMode.transitPose); Robot.manipulator.setSpeed(0);})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    public static boolean isAlignCommandFinsihed() {
        return RobotMode.alignWithAlgae.isFinished();
    }

    public static boolean isElevatorAtPosition() {
        return Robot.elevator.getPosition() <= 0.1;
    }

    public static boolean isManipulatorTouchingAlgae() {
        return (Robot.arm.getMotionMagicError() > 0.01 || Robot.arm.getMotionMagicError() < -0.01) && Robot.elevator.getPosition() <= 0.85;
    }

}
