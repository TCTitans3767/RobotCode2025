package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIntakeWheelSpeed extends Command{
    
    double speed;

    public SetIntakeWheelSpeed(double speed) {
        this.speed = speed;
        addRequirements(Robot.intake);
    }

    @Override
    public void initialize() {
        Robot.intake.setWheelSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
