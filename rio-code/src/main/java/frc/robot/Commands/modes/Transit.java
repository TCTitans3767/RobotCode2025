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
            Robot.robotMode.setCurrentMode(RobotMode.coralStationPose);
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

        if (Robot.joystick.x().getAsBoolean()) {
            Robot.intake.setWheelSpeed(-0.5);
        } else {
            Robot.intake.setWheelSpeed(0);
        }
        
        // if (TriggerBoard.isNearCoralStation() && !TriggerBoard.isCoralInManipulator()) {
        //     Robot.robotMode.setCurrentMode(RobotMode.coralStationPose);
        //     return;
        // }

        // if (TriggerBoard.isCoralButtonPressed() && !TriggerBoard.isCoralInManipulator()) {
        //     Robot.robotMode.setCurrentMode(RobotMode.coralFloorPose);
        //     return;
        // }

        // if (TriggerBoard.isCoralButtonPressed() && TriggerBoard.isCoralInManipulator()) {
        //     Robot.robotMode.setCurrentMode(RobotMode.coralReefPose);
        // }

        // if (Robot.joystick.leftTrigger(0.08).getAsBoolean()) {
        //     Robot.elevator.setSpeed(Robot.joystick.getLeftTriggerAxis() * 0.75);
        // } else if (Robot.joystick.rightTrigger(0.08).getAsBoolean()) {
        //     Robot.elevator.setSpeed(-Robot.joystick.getRightTriggerAxis() * 0.75);
        // } else {
        //     Robot.elevator.setSpeed(0);
        // }

        // if (Robot.joystick.a().getAsBoolean()) {
        //     Robot.climber.setSpeed(-0.3);
        // } else if (Robot.joystick.y().getAsBoolean()) {
        //     Robot.climber.setSpeed(0.3);
        // } else {
        //     Robot.climber.setSpeed(0);
        // }

        // if (Robot.joystick.rightBumper().getAsBoolean()) {
        //     Robot.intake.setPivotPosition(0.45);
        // }

        // if (Robot.joystick.rightBumper().getAsBoolean()) {
        //     Robot.intake.setWheelSpeed(0.75);
        // } else {
        //     Robot.intake.setWheelSpeed(0);
        // }

        // if (Robot.joystick.povDown().getAsBoolean()) {
        //     Robot.robotMode.setDriveMode(DriveMode.Brake);
        //     Robot.intake.setPivotSpeed(Robot.joystick.getRightY() * 0.3);
        // } else {
        //     Robot.intake.setPivotSpeed(0);
        //     Robot.robotMode.setDriveMode(DriveMode.TeleopDrive);
        // }

        // if (Robot.joystick.b().getAsBoolean()) {
        //     Robot.intake.setWheelSpeed(0.8);
        // } else {
        //     Robot.intake.setWheelSpeed(0);
        // }

        // if (Robot.joystick.povUp().getAsBoolean()) {
        //     Robot.robotMode.setDriveMode(DriveMode.Brake);
        //     Robot.elevator.setSpeed(Robot.joystick.getRightY() * 0.2);
        // } else {
        //     Robot.elevator.setSpeed(0);
        //     Robot.robotMode.setDriveMode(DriveMode.TeleopDrive);
        // }
  
        // if (Robot.joystick.povUp().getAsBoolean()) {
        //     Robot.robotMode.setDriveMode(DriveMode.Brake);
        //     Robot.elevator.setSpeed(Robot.joystick.getRightY() * 0.2);
        // } else {
        //     Robot.elevator.setSpeed(0);
        //     Robot.robotMode.setDriveMode(DriveMode.TeleopDrive);
        // }

        // if (Robot.joystick.rightBumper().getAsBoolean()) {
        //     Robot.arm.setPositon(-0.45);
        // }
        
        // if (Robot.joystick.start().getAsBoolean()) {
        //     Robot.arm.setPositon(0.29);
        // }

        // if (Robot.joystick.rightBumper().getAsBoolean()) {
        //     Robot.robotMode.setCurrentMode(RobotMode.ejectCoralPose);
        // }

        // if (Robot.joystick.rightBumper().getAsBoolean()) {
        //     Robot.elevator.setPosition(0.5);
        // }

        // if (Robot.joystick.start().getAsBoolean()) {
        //     Robot.elevator.setPosition(0.1);
        // }

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
