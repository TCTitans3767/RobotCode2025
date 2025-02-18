package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.drive.ControllerDrive;
import frc.robot.subsystems.RobotMode;

public class TransitPose extends Command{
    
    public TransitPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        if (TriggerBoard.isCoralInManipulator() && !TriggerBoard.isAlgaeInIntake()) {
            Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);
            Robot.manipulator.setSpeed(0);
            Robot.arm.setPosition(0.128);
            Robot.elevator.setPosition(0.5);
        } else if (TriggerBoard.isCoralInManipulator() && TriggerBoard.isAlgaeInIntake()) {
            Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);
            Robot.manipulator.setSpeed(0);
            Robot.arm.setPosition(-0.128);
            Robot.elevator.setPosition(0.5);
        } else if (TriggerBoard.isAlgaeInIntake() && !TriggerBoard.isCoralInManipulator()) {
            Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);
            Robot.manipulator.setSpeed(0);
            Robot.arm.setPosition(-0.128);
            Robot.elevator.setPosition(0.5);
        } else {
            System.out.println("Transit Pose Intialize: controllerDrive\n arm @ 0\n elevator @ 0.5m");
            Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);
            Robot.arm.setPosition(0.128);
            Robot.elevator.setPosition(0.5);
        }
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        // return Robot.arm.atPosition() && Robot.elevator.atPosition();
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setCurrentMode(RobotMode.transit);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
