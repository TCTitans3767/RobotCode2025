package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class SetModeCommand extends InstantCommand{
    
    public SetModeCommand(Command newMode) {
        super(() -> {
                Robot.robotMode.setCurrentMode(newMode);
            },
            Robot.robotMode
        );
    }

}
