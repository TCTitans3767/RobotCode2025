package frc.robot.Commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;

public class FinalClimb extends Command{
    
    public FinalClimb() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {
        if (TriggerBoard.isClimbButtonPressed()) {
            Robot.climber.setSpeed(0.8);
        } else {
            Robot.climber.setSpeed(0);
        }
    }

}
