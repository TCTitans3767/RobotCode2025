package frc.robot.Commands.arm;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetArmSpeed extends Command{

    double speed = 0;
    
    public SetArmSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        Robot.arm.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.arm.setSpeed(0);
        Robot.arm.setPosition(Robot.arm.getPosition());
    }

}
