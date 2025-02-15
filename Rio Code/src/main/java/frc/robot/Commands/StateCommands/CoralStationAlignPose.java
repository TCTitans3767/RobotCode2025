package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.DriveCommands.AlignWithCoralStation;
import frc.robot.subsystems.RobotMode;
import frc.robot.Robot;

public class CoralStationAlignPose extends Command{

    private boolean isFinished = false;

    public CoralStationAlignPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        RobotMode.alignWithCoralStation.schedule();
        Robot.manipulator.setSpeed(0.5);
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setCurrentMode(RobotMode.coralStation);
    }
    
}
