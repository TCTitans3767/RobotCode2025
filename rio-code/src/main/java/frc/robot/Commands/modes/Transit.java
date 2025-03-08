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

        if (TriggerBoard.isCoralButtonPressed() && !TriggerBoard.isCoralInManipulator()) {
            // if (TriggerBoard.isNearCoralStation()) {
                Robot.robotMode.setCurrentMode(RobotMode.coralStationPose);
            // } else {
            //     Robot.robotMode.setCurrentMode(RobotMode.coralFloorPose);
            // }
            return;
        } else if (TriggerBoard.isCoralButtonPressed() && TriggerBoard.isCoralInManipulator() && !Robot.arm.isNear(-0.5)) {
            Robot.robotMode.setCurrentMode(RobotMode.transitPose);
            return;
        } else if (TriggerBoard.isCoralButtonPressed() && TriggerBoard.isCoralInManipulator() && Robot.arm.isNear(-0.5)) {
            Robot.robotMode.setCurrentMode(RobotMode.coralReefAlignPose);
        }

        if (TriggerBoard.isAlgaeButtonPressed() && !TriggerBoard.isAlgaeInIntake()) {
            Robot.robotMode.setCurrentMode(RobotMode.algaePickupPose);
            return;
        } else if (TriggerBoard.isAlgaeButtonPressed() && TriggerBoard.isAlgaeInIntake()) {
            Robot.robotMode.setCurrentMode(RobotMode.ejectAlgaePose);
        }

        if (TriggerBoard.isCoralOverrideButtonPressed() && !TriggerBoard.isCoralInManipulator()) {
            Robot.robotMode.setCurrentMode(RobotMode.coralFloorPose);
            return;
        } else if (TriggerBoard.isCoralOverrideButtonPressed() && TriggerBoard.isCoralInManipulator()) {
            Robot.robotMode.setCurrentMode(RobotMode.transitPose);
        }

        if (Robot.joystick.b().getAsBoolean()) {
            Robot.robotMode.setCurrentMode(RobotMode.knockOffAlgaePoseManual);
        }

        
        // if (TriggerBoard.isNearCoralStation() && !TriggerBoard.isCoralInManipulator()) {
        //     Robot.robotMode.setCurrentMode(RobotMode.coralStationPose);
        //     return;
        // }

        // if (Robot.joystick.x().getAsBoolean()) {
        //     Robot.intake.setWheelSpeed(-0.5);
        // } else {
        //     Robot.intake.setWheelSpeed(0);
        // }

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
