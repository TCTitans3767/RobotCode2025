package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.DriveCommands.ControllerDrive;
import frc.robot.subsystems.RobotMode;

public class TransitPose extends Command{
    
    public TransitPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        if (TriggerBoard.isCoralInManipulator()) {
            Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);
            Robot.manipulator.setSpeed(0);
            Robot.arm.setPositon(0.128);
            Robot.elevator.setPosition(0.5);
        } else {
            System.out.println("Transit Pose Intialize: controllerDrive\n arm @ 0\n elevator @ 0.5m");
            Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);
            Robot.arm.setPositon(0.128);
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
