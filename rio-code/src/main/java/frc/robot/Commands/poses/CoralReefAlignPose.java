package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ButtonBox;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralReefAlignPose extends SequentialCommandGroup{

    Command leftReefAlign = new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.alignWithLeftReef);});
    Command rightReefAlign = new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.alignWithRightReef);});
    
    public CoralReefAlignPose() {

        addCommands(
            new ConditionalCommand(leftReefAlign, rightReefAlign, ButtonBox::isLeftBranchSelected),
            new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);}),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(RobotMode.coralReefAligned);})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }
}
