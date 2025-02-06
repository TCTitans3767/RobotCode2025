package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Commands.StateCommands.Idle;
import frc.robot.Commands.StateCommands.ScoreLeft;
import frc.robot.Commands.StateCommands.ScoreRight;
import frc.robot.utils.Logger;

public class RobotController extends SubsystemBase {

    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Manipulator manipulator;
    private final Climber climber;
    private final Limelight lineupCamera;
    private final Arm arm;
    private final Intake intake;

    private final Idle idle;
    private final ScoreLeft scoreLeft;
    private final ScoreRight scoreRight;

    public RobotController(Drivetrain drivetrain, Elevator elevator, Manipulator manipulator, Arm arm, Intake intake, Climber climber, Limelight lineupCamera, Supplier<SwerveRequest> request) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.manipulator = manipulator;
        this.climber = climber;
        this.arm = arm;
        this.intake = intake;
        this.lineupCamera = lineupCamera;

        idle = new Idle(this, drivetrain, elevator, manipulator, arm, intake, climber, request);
        scoreLeft = new ScoreLeft(this, drivetrain, elevator, manipulator, arm, intake, climber, lineupCamera);
        scoreRight = new ScoreRight(this, drivetrain, elevator, manipulator, arm, intake, climber, lineupCamera);
        this.setDefaultCommand(idle);
    }

    @Override
    public void periodic() {

        Logger.log("RobotController/Active Command", this.getCurrentCommand().getName());

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

    public void cancelAll() {
        CommandScheduler.getInstance().cancelAll();
    }
    
}
