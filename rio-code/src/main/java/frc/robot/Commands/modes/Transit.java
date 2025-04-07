package frc.robot.Commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;

public class Transit extends Command{

    
    public Transit() {
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

        if (Robot.intake.isWheelMotorTooHot()) {
            Robot.intake.setWheelSpeed(0);
        }

        if (!TriggerBoard.isAlgaeInIntake()) {
            Robot.intake.setWheelSpeed(0);
        }

        if (TriggerBoard.isCoralButtonPressed()) {

            if (TriggerBoard.isCoralInManipulator()) {

                if (TriggerBoard.isL1Selected()) {
                    Robot.robotMode.setCurrentMode(RobotMode.coralReefPose);
                    return;
                } else {
                    Robot.robotMode.setCurrentMode(RobotMode.coralReefAlignPose);
                    return;
                }

            } else if (!TriggerBoard.isCoralInManipulator()) {
                Robot.robotMode.setCurrentMode(RobotMode.coralStationPose);
                return;
            }

        } else if (TriggerBoard.isCoralOverrideButtonPressed()) {

            if (!TriggerBoard.isCoralInManipulator()) {
                Robot.robotMode.setCurrentMode(RobotMode.coralFloorPose);
                return;
            } else if (TriggerBoard.isCoralInManipulator()) {
                Robot.robotMode.setCurrentMode(RobotMode.transitPose);
                return;
            }

        } else if (TriggerBoard.isAlgaeButtonPressed()) {

            if (!TriggerBoard.isAlgaeInIntake()) {
                Robot.robotMode.setCurrentMode(RobotMode.algaePickupPose);
                return;
            } else if (TriggerBoard.isAlgaeInIntake()) {
                Robot.robotMode.setCurrentMode(RobotMode.ejectAlgaePose);
                return;
            }
            
        }

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
