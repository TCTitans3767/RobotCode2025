package frc.robot.Commands.panic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TriggerBoard;

public class PanicIntake extends Command {
    public PanicIntake() {
        addRequirements(Robot.intake);
    }

    @Override
    public void execute() {
        double intakePivotSpeed = Robot.panicController.getRightY() * Constants.Panic.intakeStickMaxSpeed;
        double rollerSpeed = 0;

        if (TriggerBoard.isPanicButtonHighPressed()) {
            rollerSpeed = Constants.Panic.intakeSpeed;
        } else if (TriggerBoard.isPanicButtonLowPressed()) {
            rollerSpeed = -Constants.Panic.intakeSpeed;
        }

        Robot.intake.setPivotPosition(intakePivotSpeed);
        Robot.intake.setWheelSpeed(rollerSpeed);
    }
}
