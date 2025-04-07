package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIntakeWheelPower extends Command{
    
    double speed;

    public SetIntakeWheelPower(double speed) {
        this.speed = speed;
        addRequirements(Robot.intake);
    }

    @Override
    public void initialize() {
        Robot.intake.setWheelPower(speed);
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
