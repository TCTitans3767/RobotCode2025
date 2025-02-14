package frc.robot.Commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class CoralFloor extends Command{
    
    public CoralFloor() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator);
    }

    @Override
    public void execute() {
        if (TriggerBoard.isCoralInManipulator()) {
            
        } else if (TriggerBoard.isCoralButtonPressed()) {
            Robot.robotMode.setCurrentMode(RobotMode.transitPose);
        }
    }

}
