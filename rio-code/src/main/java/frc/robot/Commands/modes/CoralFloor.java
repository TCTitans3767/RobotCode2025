package frc.robot.Commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class CoralFloor extends Command{
    
    public CoralFloor() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (TriggerBoard.isL1Selected() && TriggerBoard.isCoralInIntake()) {
            Robot.robotMode.setCurrentMode(RobotMode.transitPose);
        }

        if (TriggerBoard.isCoralInManipulator()) {
            Robot.robotMode.setCurrentMode(RobotMode.coralReefPose);
        } else if (TriggerBoard.isCoralOverrideButtonPressed()) {
            Robot.robotMode.setCurrentMode(RobotMode.transitPose);
        }
    }

}
