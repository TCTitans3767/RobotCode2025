package frc.robot.Commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode;

public class CoralFloorPose extends Command{
    
    public CoralFloorPose() {
        addRequirements(Robot.intake, Robot.arm, Robot.manipulator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setCurrentMode(RobotMode.coralFloor);
    }

}
