package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class PanicIntakeWheelSpeed extends Command{
    
    private final double speed;

    private double previousSpeed;

    public PanicIntakeWheelSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        previousSpeed = 0;
        previousSpeed = Robot.intake.getWheelSpeed();

        Robot.intake.setWheelSpeed(speed);

    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.setWheelSpeed(previousSpeed);
    }

}
