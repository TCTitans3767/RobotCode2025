package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode;

public class SetDriveModeCommand extends InstantCommand{
    
    public SetDriveModeCommand(Command newDriveCommand) {
        super(() -> {
                Robot.robotMode.setDriveModeCommand(newDriveCommand);
            }, 
            Robot.robotMode
        );
    }

}
