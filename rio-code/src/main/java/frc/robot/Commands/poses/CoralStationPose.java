package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class CoralStationPose extends Command{

    public CoralStationPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        if (!TriggerBoard.isAutonActive()) {
            Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);
        }
        Robot.arm.setPosition(0.128);
        Robot.manipulator.setSpeed(-0.2);
        Robot.elevator.setPosition(0.5);
    }

    @Override
    public boolean isFinished() {
        return Robot.arm.isAtPosition() && Robot.elevator.isAtPosition();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setCurrentMode(RobotMode.coralStation);
    }
    
}
