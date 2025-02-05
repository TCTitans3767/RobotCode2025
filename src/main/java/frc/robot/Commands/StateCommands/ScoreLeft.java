package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.DriveCommands.AlignWithLeftReef;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.RobotController;

public class ScoreLeft extends Command{

    private final RobotController robotController;
    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Manipulator manipulator;
    private final Climber climber;
    private final Arm arm;
    private final Intake intake;

    private final AlignWithLeftReef alignWithLeftReef;
    private final Limelight camera;

    public ScoreLeft(RobotController robotController, Drivetrain drivetrain, Elevator elevator, Manipulator manipulator, Arm arm, Intake intake, Climber climber, Limelight camera) {
        this.camera = camera;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.manipulator = manipulator;
        this.climber = climber;
        this.arm = arm;
        this.intake = intake;
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
