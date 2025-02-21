package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotMode;
import frc.robot.Robot;
import frc.robot.Commands.drive.AlignWithCoralStation;

public class CoralStationAlignPose extends Command{

    public CoralStationAlignPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        Robot.robotMode.setDriveModeCommand(RobotMode.alignWithCoralStation);
        Robot.manipulator.setSpeed(-0.2);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setCurrentMode(RobotMode.coralStation);
    }
    
}
