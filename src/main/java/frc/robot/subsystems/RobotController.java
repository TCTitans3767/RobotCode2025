package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Commands.StateCommands.Idle;
import frc.robot.Commands.StateCommands.ScoreLeft;
import frc.robot.Commands.StateCommands.ScoreRight;

public class RobotController extends SubsystemBase {

    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Manipulator manipulator;
    private final Climber climber;
    private final Limelight lineupCamera;

    private final Idle idle;
    private final ScoreLeft scoreLeft;
    private final ScoreRight scoreRight;

    public RobotController(Drivetrain drivetrain, Elevator elevator, Manipulator manipulator, Climber climber, Limelight lineupCamera, Supplier<SwerveRequest> request) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.manipulator = manipulator;
        this.climber = climber;
        this.lineupCamera = lineupCamera;

        idle = new Idle(this, drivetrain, elevator, manipulator, climber, request);
        scoreLeft = new ScoreLeft(this, lineupCamera);
        scoreRight = new ScoreRight(this, lineupCamera);
        this.setDefaultCommand(idle);
    }

    @Override
    public void periodic() {

    }

    public void idle() {
        idle.schedule();
    }

    public ScoreLeft ScoreLeft() {
        return scoreLeft;
    }

    public ScoreRight ScoreRight() {
        return scoreRight;
    }
    
}
