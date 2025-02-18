package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class CoralScorePose extends Command{
    
    public CoralScorePose() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        Robot.manipulator.setSpeed(0.5);
    }

    @Override
    public boolean isFinished() {
        return !TriggerBoard.isCoralInManipulator();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setCurrentMode(RobotMode.transitPose);
    }

}
