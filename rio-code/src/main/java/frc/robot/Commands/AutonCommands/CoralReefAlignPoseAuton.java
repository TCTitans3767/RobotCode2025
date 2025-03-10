package frc.robot.Commands.AutonCommands;

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
import frc.robot.ButtonBox;
import frc.robot.DashboardButtonBox;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;
import frc.robot.utils.Utils;
import frc.robot.utils.Utils.ReefPosition;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralReefAlignPoseAuton extends SequentialCommandGroup{

    Command leftReefAlign;
    Command rightReefAlign;

    private final boolean leftAlign;
    
    public CoralReefAlignPoseAuton(ReefPosition targetReef, String reefLevel, boolean leftAlign) {

        this.leftAlign = leftAlign;

        leftReefAlign = new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(new AlignWithLeftReefAuton(targetReef));});
        rightReefAlign = new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(new AlignWithRightReefAuton(targetReef));});

        addCommands(
            new ConditionalCommand(leftReefAlign, rightReefAlign, () -> leftAlign),
            new WaitUntilCommand(() -> isAlignCommandFinsihed()),
            new InstantCommand(() -> {Robot.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0));}),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(new CoralReefPoseAuton(reefLevel));})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    public static boolean isLeftBranchSelected() {
        return (DashboardButtonBox.getSelectedReefBranch() == ReefPosition.A || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.C ||DashboardButtonBox.getSelectedReefBranch() == ReefPosition.E || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.G || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.I || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.K);
    }

    public boolean isAlignCommandFinsihed() {
        return Robot.robotMode.isDriveCommandFinished();
    }
}
