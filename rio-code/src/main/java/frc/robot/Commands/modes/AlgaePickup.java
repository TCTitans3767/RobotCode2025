package frc.robot.Commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class AlgaePickup extends Command{
    
    public AlgaePickup() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {
        if (TriggerBoard.isAlgaeInIntake()) {
            Robot.robotMode.setCurrentMode(RobotMode.transitPose);
        } else if (TriggerBoard.isAlgaeButtonPressed()) {
            Robot.robotMode.setCurrentMode(RobotMode.transitPose);
        }
    }

}
