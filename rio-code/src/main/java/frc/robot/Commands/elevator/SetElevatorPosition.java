package frc.robot.Commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Utils;

public class SetElevatorPosition extends Command{

    private double positionMeters = 0;

    public SetElevatorPosition(double positionMeters) {
        this.positionMeters = positionMeters;
        addRequirements(Robot.elevator);
    }

    @Override
    public void initialize() {
        Robot.elevator.setPosition(Utils.metersToRotations(positionMeters, Constants.Elevator.RotationsPerMeter));
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Robot.elevator.isAtPosition();
    }
    
}
