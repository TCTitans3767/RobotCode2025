package frc.robot.Commands.modes;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DashboardButtonBox;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class CoralReef extends Command{
    
    public CoralReef() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        // if (DashboardButtonBox.hasSelectedLevelChanged()) {
        //     Robot.robotMode.setCurrentMode(RobotMode.coralReefPose);
        // }

        if (TriggerBoard.isL1Selected() && TriggerBoard.isCoralButtonPressed()) {
            Robot.robotMode.setCurrentMode(RobotMode.scoreCoralPose);
            return;
        }

        if (TriggerBoard.isCoralButtonPressed() && TriggerBoard.isNearReef()) {
            Robot.robotMode.setCurrentMode(RobotMode.coralReefAlignPose);
            return;
        } 

        if (TriggerBoard.isCoralOverrideButtonPressed()) {
            Robot.robotMode.setCurrentMode(RobotMode.scoreCoralPose);
            return;
        }

        // if (TriggerBoard.isCoralButtonPressed() && TriggerBoard.isNearReef()) {
        //     Robot.robotMode.setCurrentMode(RobotMode.coralReefAlignPose);
        //     return;
        // } 
        //   else if (TriggerBoard.isCoralButtonPressed() && !TriggerBoard.isNearReef()) {
        //     Robot.joystick.setRumble(RumbleType.kBothRumble, 1);
        //     Robot.joystick.setRumble(RumbleType.kBothRumble, 0);
        //     return;
        // }

        // if (!TriggerBoard.isNearReef()) {
        //     Robot.robotMode.setCurrentMode(RobotMode.transitPose);
        // }

    }

}
