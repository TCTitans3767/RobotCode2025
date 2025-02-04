package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.DriveCommands.AlignWithLeftReef;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotController;

public class ScoreLeft extends Command{

    private final RobotController robotController;

    private final AlignWithLeftReef alignWithLeftReef;
    private final Limelight camera;

    public ScoreLeft(RobotController robotController, Limelight camera) {
        this.camera = camera;
        this.robotController = robotController;
        alignWithLeftReef = new AlignWithLeftReef(camera);

        addRequirements(robotController);
    }

    @Override
    public void initialize() {
        alignWithLeftReef.schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        alignWithLeftReef.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
