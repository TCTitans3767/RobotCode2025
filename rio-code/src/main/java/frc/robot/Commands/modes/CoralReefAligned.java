package frc.robot.Commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class CoralReefAligned extends Command{

    public CoralReefAligned() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {
        if (TriggerBoard.isCoralButtonPressed()) {
            Robot.robotMode.setCurrentMode(RobotMode.coralScorePose);
        }
    }

}