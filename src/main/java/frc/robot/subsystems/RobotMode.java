package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Commands.ZeroElevator;
import frc.robot.Commands.StateCommands.DoNothing;
import frc.robot.Commands.StateCommands.Idle;
import frc.robot.Commands.StateCommands.ScoreLeft;
import frc.robot.Commands.StateCommands.ScoreRight;
import frc.robot.utils.Logger;

public class RobotMode extends SubsystemBase {

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

    private final ZeroElevator zeroElevator;
    private final DoNothing doNothing;

    public RobotMode(Supplier<SwerveRequest> request) {
        drivetrain = Robot.drivetrain;
        elevator = Robot.elevator;
        manipulator = Robot.manipulator;
        climber = Robot.climber;
        arm = Robot.arm;
        intake = Robot.intake;
        lineupCamera = Robot.limelight;

        zeroElevator = new ZeroElevator(elevator);
        doNothing = new DoNothing(this);

        idle = new Idle(this, request);
        scoreLeft = new ScoreLeft(this);
        scoreRight = new ScoreRight(this, drivetrain, elevator, manipulator, arm, intake, climber, lineupCamera);
        this.setDefaultCommand(idle);
    }

    @Override
    public void periodic() {

        // Logger.log("RobotController/Active Command", this.getCurrentCommand().getName());

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

    public void resetAllEncoders() {
        doNothing.schedule();
        new SequentialCommandGroup(
            zeroElevator
        ).andThen(() -> {
            doNothing.cancel();
        });
    }
    
}
