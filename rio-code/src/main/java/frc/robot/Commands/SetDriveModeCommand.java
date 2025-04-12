package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode;

public class SetDriveModeCommand extends Command{

    Command newDriveCommand;
    
    public SetDriveModeCommand(Command newDriveCommand) {
        this.newDriveCommand = newDriveCommand;
    }

    @Override
    public void initialize() {
        Robot.robotMode.setDriveModeCommand(newDriveCommand);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
