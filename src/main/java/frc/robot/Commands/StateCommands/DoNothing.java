package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode;

public class DoNothing extends Command{
    
    private final RobotMode robotController = Robot.robotMode;
    
    public DoNothing() {
        addRequirements(robotController);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
    
}
