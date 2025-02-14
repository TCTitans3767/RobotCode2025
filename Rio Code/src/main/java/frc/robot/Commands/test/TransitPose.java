package frc.robot.Commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class TransitPose extends Command{
    
    public TransitPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator);
    }

    @Override
    public void initialize() {
        if (TriggerBoard.isCoralInManipulator()) {
            Robot.arm.setPositon(0);
            Robot.elevator.setPosition(0.5);
        } else {
            Robot.arm.setPositon(0);
            Robot.elevator.setPosition(0.5);
        }
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return Robot.arm.atPosition() && Robot.elevator.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setCurrentMode(RobotMode.transit);
    }

}
