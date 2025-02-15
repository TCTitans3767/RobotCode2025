package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;

public class Transit extends Command{

    
    public Transit() {
        addRequirements(Robot.arm, Robot.climber, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {
        
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

        if (Robot.joystick.leftTrigger(0.08).getAsBoolean()) {
            Robot.elevator.setSpeed(Robot.joystick.getLeftTriggerAxis() * 0.75);
        } else if (Robot.joystick.rightTrigger(0.08).getAsBoolean()) {
            Robot.elevator.setSpeed(-Robot.joystick.getRightTriggerAxis() * 0.75);
        } else {
            Robot.elevator.setSpeed(0);
        }

        if (Robot.joystick.a().getAsBoolean()) {
            Robot.climber.setSpeed(-0.3);
        } else if (Robot.joystick.y().getAsBoolean()) {
            Robot.climber.setSpeed(0.3);
        } else {
            Robot.climber.setSpeed(0);
        }

        if (Robot.joystick.povDown().getAsBoolean()) {
            Robot.robotMode.setDriveMode(DriveMode.Brake);
            Robot.arm.setSpeed(Robot.joystick.getRightY() * 0.1);
        } else {
            Robot.arm.setSpeed(0);
            Robot.robotMode.setDriveMode(DriveMode.TeleopDrive);
        }

        if (Robot.joystick.rightBumper().getAsBoolean()) {
            Robot.robotMode.setCurrentMode(RobotMode.ejectCoralPose);
        }

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
