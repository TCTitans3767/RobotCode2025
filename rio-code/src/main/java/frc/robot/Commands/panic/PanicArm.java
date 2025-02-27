package frc.robot.Commands.panic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TriggerBoard;

public class PanicArm extends Command {
    public PanicArm() {
        addRequirements(Robot.arm, Robot.manipulator);
    }

    @Override
    public void execute() {
        double armSpeed = Robot.panicController.getRightY() * Constants.Panic.armStickMaxSpeed;
        double manipulatorSpeed = 0;

        if (TriggerBoard.isPanicButtonHighPressed()) {
            manipulatorSpeed = Constants.Panic.manipulatorSpeed;
        } else if (TriggerBoard.isPanicButtonLowPressed()) {
            manipulatorSpeed = -Constants.Panic.manipulatorSpeed;
        }

        Robot.arm.setSpeed(armSpeed);
        Robot.manipulator.setSpeed(manipulatorSpeed);
    }
}
