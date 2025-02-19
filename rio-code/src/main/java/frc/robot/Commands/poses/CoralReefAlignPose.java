package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Commands.SetDriveModeCommand;
import frc.robot.Commands.SetModeCommand;
import frc.robot.subsystems.RobotMode;
import frc.robot.utils.Utils;
import frc.robot.utils.Utils.ReefPosition;

public class CoralReefAlignPose extends SequentialCommandGroup{
    
    public CoralReefAlignPose() {

        addCommands(
            new ConditionalCommand(new SetDriveModeCommand(RobotMode.alignWithLeftReef), new SetDriveModeCommand(RobotMode.alignWithRightReef), CoralReefAlignPose::isLeftBranchSelected),
            new SetDriveModeCommand(RobotMode.controllerDrive),
            new SetModeCommand(RobotMode.coralReefAligned)
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    public static boolean isLeftBranchSelected() {
        return (Utils.getSelectedReefPosition() == ReefPosition.A || Utils.getSelectedReefPosition() == ReefPosition.C || Utils.getSelectedReefPosition() == ReefPosition.E || Utils.getSelectedReefPosition() == ReefPosition.G || Utils.getSelectedReefPosition() == ReefPosition.I || Utils.getSelectedReefPosition() == ReefPosition.K);
    }

}
