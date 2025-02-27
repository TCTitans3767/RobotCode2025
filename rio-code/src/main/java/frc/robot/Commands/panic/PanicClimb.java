package frc.robot.Commands.panic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class PanicClimb extends Command {
    public PanicClimb() {
        addRequirements(Robot.elevator);
    }

    @Override
    public void execute() {
        double elevatorSpeed = Robot.panicController.getRightY() * Constants.Panic.elevatorStickMaxSpeed;

        Robot.elevator.setSpeed(elevatorSpeed);
    }
}


