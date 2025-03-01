package frc.robot.Commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetClimberSpeed extends Command{
    
    double speed = 0;

    public SetClimberSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        Robot.climber.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.setSpeed(0);
    }
    
}
