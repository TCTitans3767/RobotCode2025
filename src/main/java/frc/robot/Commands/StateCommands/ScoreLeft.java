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

    private final RobotMode robotController = Robot.robotMode;
    private final Drivetrain drivetrain = Robot.drivetrain;
    private final Elevator elevator = Robot.elevator;
    private final Manipulator manipulator = Robot.manipulator;
    private final Climber climber = Robot.climber;
    private final Arm arm = Robot.arm;
    private final Intake intake = Robot.intake;

    private final AlignWithLeftReef alignWithLeftReef = new AlignWithLeftReef();
    private final Limelight camera = Robot.limelight;

    public ScoreLeft() {
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
