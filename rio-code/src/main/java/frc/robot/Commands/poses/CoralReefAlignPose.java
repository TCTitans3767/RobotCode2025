package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ButtonBox;
import frc.robot.DashboardButtonBox;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;
import frc.robot.utils.Utils;
import frc.robot.utils.Utils.ReefPosition;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralReefAlignPose extends SequentialCommandGroup{

    Command leftReefAlign = new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.alignWithLeftReef);});
    Command rightReefAlign = new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.alignWithRightReef);});
    
    public CoralReefAlignPose() {

        addCommands(
            new ConditionalCommand(leftReefAlign, rightReefAlign, CoralReefAlignPose::isLeftBranchSelected),
            new WaitUntilCommand(CoralReefAlignPose::isAlignCommandFinsihed).withTimeout(1.2),
            new InstantCommand(() -> {Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0));}),
            new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.slowControllerDrive);}),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(RobotMode.coralReefPose);})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    public static boolean isLeftBranchSelected() {
        return (DashboardButtonBox.getSelectedReefBranch() == ReefPosition.A || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.C ||DashboardButtonBox.getSelectedReefBranch() == ReefPosition.E || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.G || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.I || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.K);
    }

    public static boolean isAlignCommandFinsihed() {
        return CoralReefAlignPose.isLeftBranchSelected() ? RobotMode.alignWithLeftReef.isFinished() : RobotMode.alignWithRightReef.isFinished();
    }
}
