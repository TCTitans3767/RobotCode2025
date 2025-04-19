package frc.robot.Commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class L1 extends Command {

    public L1() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (TriggerBoard.isCoralInManipulator()) {
            Robot.robotMode.setCurrentMode(RobotMode.transitPose);
        }

        if (TriggerBoard.isCoralButtonPressed()) {
            Robot.robotMode.setCurrentMode(RobotMode.scoreCoralPose);
        }

    }

}
