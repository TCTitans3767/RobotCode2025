package frc.robot.Commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetManipulatorWheelSpeed extends Command{
    
    double speed;

    public SetManipulatorWheelSpeed(double speed) {
        this.speed = speed;
        addRequirements(Robot.manipulator);
    }

    @Override
    public void initialize() {
        Robot.manipulator.setSpeed(speed);
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
