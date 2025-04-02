package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode;

public class AlgaePickupPose extends Command{
    
    public AlgaePickupPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        if (Robot.intake.isWheelMotorTooHot()) {
            this.cancel();
            return;
        } else {
            Robot.arm.setPosition(-0.1);
            Robot.intake.setWheelSpeed(0.5);
            Robot.intake.setPivotPosition(-0.07);
        }
    }

    @Override
    public boolean isFinished() {
        return Robot.intake.isPivotAtPosition();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setCurrentMode(RobotMode.algaePickup);
    }

}
