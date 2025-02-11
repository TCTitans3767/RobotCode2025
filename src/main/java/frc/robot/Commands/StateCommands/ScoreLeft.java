package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Commands.DriveCommands.AlignWithLeftReef;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.RobotMode;

public class ScoreLeft extends Command{

    private final RobotMode robotController;
    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Manipulator manipulator;
    private final Climber climber;
    private final Arm arm;
    private final Intake intake;

    private final AlignWithLeftReef alignWithLeftReef;
    private final Limelight camera;

    public ScoreLeft(RobotMode robotController) {
        camera = Robot.limelight;
        drivetrain = Robot.drivetrain;
        elevator = Robot.elevator;
        manipulator = Robot.manipulator;
        climber = Robot.climber;
        arm = Robot.arm;
        intake = Robot.intake;
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
