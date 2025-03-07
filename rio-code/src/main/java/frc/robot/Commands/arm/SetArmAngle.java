package frc.robot.Commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetArmAngle extends Command{

    private double angle;

    public SetArmAngle(double angle) {
        this.angle = angle;
        addRequirements(Robot.arm);
    }

    @Override
    public void initialize() {
        Robot.arm.setPosition(angle);
    }

    @Override
    public boolean isFinished() {
        return Robot.arm.isAtPosition();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}
