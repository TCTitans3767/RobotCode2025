package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode;

public class CoralStationPose extends Command{

    public CoralStationPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);
        Robot.arm.setPositon(90);
        Robot.elevator.setPosition(0.5);
    }

    @Override
    public boolean isFinished() {
        return Robot.arm.atPosition() && Robot.elevator.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || (!Robot.manipulator.hasGamePiece())) {
            RobotMode.coralStation.cancel();
        }
        // RobotMode.CoralTransitPose.schedule();
    }
    
}
