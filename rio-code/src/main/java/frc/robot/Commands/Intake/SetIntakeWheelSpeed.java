package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIntakeWheelSpeed extends Command{
    
    double rotationsPerSecond;

    public SetIntakeWheelSpeed(double rotationsPerSecond) {
        this.rotationsPerSecond = rotationsPerSecond;
        addRequirements(Robot.intake);
    }

    @Override
    public void initialize() {
        Robot.intake.setWheelSpeed(rotationsPerSecond);
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
