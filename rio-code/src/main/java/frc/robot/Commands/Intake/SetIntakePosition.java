package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIntakePosition extends Command{

    private double position;
    
    public SetIntakePosition(double position) {
        this.position = position;
        addRequirements(Robot.intake);
    }

    @Override
    public void initialize() {
        Robot.intake.setPivotPosition(position);
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
