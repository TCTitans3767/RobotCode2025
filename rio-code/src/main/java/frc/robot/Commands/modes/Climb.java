package frc.robot.Commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class Climb extends Command{
    
    public Climb() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (!TriggerBoard.isClimbButtonBoxButtonPressed()) {
            Robot.robotMode.setCurrentMode(RobotMode.initialTransitPose);
        }

        if (TriggerBoard.isClimbControllerButtonPressed()) {
            Robot.robotMode.setCurrentMode(RobotMode.deployClimberPose);
        }
    }

}
