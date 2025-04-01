package frc.robot.Commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetClimberRotations extends Command{
    
    double rotations = 0;

    public SetClimberRotations(double rotations) {
        this.rotations = rotations;
    }

    @Override
    public void initialize() {
        Robot.climber.setRotations(rotations);
    }

    @Override
    public boolean isFinished() {
        return Robot.climber.isAtPosition();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.setSpeed(0);
    }
    
}
