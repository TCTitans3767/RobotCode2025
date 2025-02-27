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

        if (TriggerBoard.isCoralButtonPressed() && !TriggerBoard.isCoralInManipulator()) {
            if (TriggerBoard.isNearCoralStation()) {
                Robot.robotMode.setCurrentMode(RobotMode.coralStationPose);
            } else {
                Robot.robotMode.setCurrentMode(RobotMode.coralFloorPose);
            }
        } else if (TriggerBoard.isCoralButtonPressed() && TriggerBoard.isCoralInManipulator()) {
            Robot.robotMode.setCurrentMode(RobotMode.coralReefPose);
        }
        
        if (Robot.joystick.a().getAsBoolean()) {
            Robot.climber.setSpeed(0.6);
        } else if (Robot.joystick.y().getAsBoolean()) {
            Robot.climber.setSpeed(-0.6);
        } else {
            Robot.climber.setSpeed(0);
        }

        if (Robot.joystick.b().getAsBoolean()) {
            Robot.robotMode.setCurrentMode(RobotMode.knockOffAlgaePose);
        }

        if (TriggerBoard.isAlgaeButtonPressed()) {
            Robot.intake.setPivotPosition(0);
        }

        if (Robot.joystick.x().getAsBoolean()) {
            Robot.intake.setPivotPosition(0.2);
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
