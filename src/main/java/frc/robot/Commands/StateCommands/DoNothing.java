package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotMode;

public class DoNothing extends Command{
    
    private final RobotMode robotController;
    
    public DoNothing(RobotMode robotController) {
        this.robotController = robotController;
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
