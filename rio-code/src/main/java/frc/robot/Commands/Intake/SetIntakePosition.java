package frc.robot.Commands.Intake;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIntakePosition extends Command{

    private final double position;
    private final double maximumVelocity;
    
    public SetIntakePosition(double position) {
        this.position = position;
        this.maximumVelocity = 0;
        addRequirements(Robot.intake);
    } 

    public SetIntakePosition(double position, double maximumVelocity) {
        this.maximumVelocity = maximumVelocity;
        this.position = position;
        addRequirements(Robot.intake);
    }

    @Override
    public void initialize() {
        if (maximumVelocity == 0) {
            Robot.intake.setPivotPosition(position);
        } else {
            Robot.intake.setPivotPosition(position, maximumVelocity);
        }
    }

    @Override
    public boolean isFinished() {
        return Robot.intake.isPivotAtPosition();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}   
