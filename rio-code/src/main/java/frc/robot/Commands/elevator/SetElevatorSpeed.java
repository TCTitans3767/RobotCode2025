package frc.robot.Commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetElevatorSpeed extends Command{
    
    double speed = 0;

    public SetElevatorSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        Robot.elevator.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.elevator.setSpeed(0);
        Robot.elevator.setPosition(Robot.elevator.getHeightMeters());
    }

}
