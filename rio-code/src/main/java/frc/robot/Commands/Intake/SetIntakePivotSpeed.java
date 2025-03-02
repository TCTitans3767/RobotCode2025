package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetIntakePivotSpeed extends Command{

    double speed = 0;
    
    public SetIntakePivotSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        Robot.intake.setPivotSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.setPivotSpeed(0);
        Robot.intake.setPivotPosition(Robot.intake.getPivotPosition());
    }

}
