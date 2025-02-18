package frc.robot.Commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends Command{

    private final Elevator elevator = Robot.elevator;
    private boolean zeroed = false;
    
    public ZeroElevator() {
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSpeed(0);
        elevator.disableSoftwareLimits();
    }

    @Override
    public void execute() {
        elevator.setSpeed(-0.05);
        if (elevator.getSpeed() > -0.01) {
            elevator.setSpeed(0);
            elevator.resetEncoder();
            zeroed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return zeroed;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.enableSoftwareLimits();
    }

}
