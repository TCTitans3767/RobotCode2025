package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode;
import frc.robot.utils.Utils;
import frc.robot.utils.Utils.ReefPosition;

public class CoralReefAlignPose extends Command{
    
    public CoralReefAlignPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        if (Utils.getSelectedReefPosition() == ReefPosition.A || Utils.getSelectedReefPosition() == ReefPosition.C || Utils.getSelectedReefPosition() == ReefPosition.E || Utils.getSelectedReefPosition() == ReefPosition.G || Utils.getSelectedReefPosition() == ReefPosition.I || Utils.getSelectedReefPosition() == ReefPosition.K) {
            Robot.robotMode.setDriveModeCommand(RobotMode.alignWithLeftReef);
        } else if (Utils.getSelectedReefPosition() == ReefPosition.B || Utils.getSelectedReefPosition() == ReefPosition.D || Utils.getSelectedReefPosition() == ReefPosition.F || Utils.getSelectedReefPosition() == ReefPosition.H || Utils.getSelectedReefPosition() == ReefPosition.J || Utils.getSelectedReefPosition() == ReefPosition.L) {
            Robot.robotMode.setDriveModeCommand(RobotMode.alignWithRightReef);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setCurrentMode(RobotMode.coralReef);
    }

}
