package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class SetModeCommand extends Command{
    
    Command newMode;

    public SetModeCommand(Command newMode) {
        this.newMode = newMode;
        addRequirements(Robot.robotMode);
    }

    @Override
    public void initialize() {
        Robot.robotMode.setCurrentMode(newMode);
    }

}
