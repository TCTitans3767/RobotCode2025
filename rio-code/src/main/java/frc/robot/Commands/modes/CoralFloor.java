package frc.robot.Commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class CoralFloor extends Command{
    
    public CoralFloor() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (TriggerBoard.isClimbButtonBoxButtonPressed()) {
            if (TriggerBoard.isClimbControllerButtonPressed()) {
                Robot.robotMode.setCurrentMode(RobotMode.climbPose);
            }
            return;
        }

        if (TriggerBoard.isL1Selected() && TriggerBoard.isCoralInIntake()) {
            Robot.robotMode.setCurrentMode(RobotMode.L1Pose);
            Robot.manipulator.setSpeed(0);
            return;
        }

        if (!TriggerBoard.isL1Selected() && TriggerBoard.isCoralInManipulator()) {
            Robot.intake.setWheelSpeed(-20);
            Robot.robotMode.setCurrentMode(RobotMode.transitPose);
        } else if (!TriggerBoard.isL1Selected() && TriggerBoard.isCoralOverrideButtonPressed()) {
            Robot.robotMode.setCurrentMode(RobotMode.transitPose);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.manipulator.setSpeed(-0.05);
        Robot.intake.setWheelSpeed(0);
    }

}
