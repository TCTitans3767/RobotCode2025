package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.ButtonBox;
import frc.robot.DashboardButtonBox;
import frc.robot.Robot;
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
            new WaitUntilCommand(CoralReefAlignPose::isAlignCommandFinsihed),
            new InstantCommand(() -> {
                Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0.6));
            }),
            new WaitCommand(0.25),
            new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);}),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(RobotMode.coralReefAligned);})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    public static boolean isLeftBranchSelected() {
        return (DashboardButtonBox.getSelectedReefBranch() == ReefPosition.A || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.C ||DashboardButtonBox.getSelectedReefBranch() == ReefPosition.E || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.G || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.I || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.K);
    }

    public static boolean isAlignCommandFinsihed() {
        return RobotMode.alignWithLeftReef.isFinished() || RobotMode.alignWithRightReef.isFinished();
    }
}
