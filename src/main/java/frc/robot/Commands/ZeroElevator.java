package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends Command{

    private final Elevator elevator;
    private boolean zeroed = false;
    
    public ZeroElevator(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSpeed(0);
    }

    @Override
    public void execute() {
        elevator.setSpeed(-0.05);
        if (elevator.getMotorTourque() > 120) {
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
        
    }

}
