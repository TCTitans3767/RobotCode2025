package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class CoralReef extends Command{
    
    public CoralReef() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {
        if (TriggerBoard.isCoralButtonPressed() && TriggerBoard.isNearReef()) {
            Robot.robotMode.setCurrentMode(RobotMode.coralReefAlignPose);
        } else if (TriggerBoard.isCoralButtonPressed() && !TriggerBoard.isNearReef()) {
            Robot.joystick.setRumble(RumbleType.kBothRumble, 1);
            Robot.joystick.setRumble(RumbleType.kBothRumble, 0);
        }
    }

}
