package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
import frc.robot.generated.TunerConstants;
import frc.robot.utils.Logger;
import pabeles.concurrency.IntOperatorTask.Max;

public class RobotMode extends SubsystemBase {

    public enum DriveMode {
        TeleopDrive,
        RobotCentric, 
        FieldCentric,
        Auton,
        Brake
    }

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private Supplier<Double> SwerveXSupplier;
    private Supplier<Double> SwerveYSupplier;
    private Supplier<Double> SwerveRotationSupplier;
    private final SwerveRequest.FieldCentric teleopDrive = new SwerveRequest.FieldCentric().withDeadband(0.02 * MaxSpeed).withRotationalDeadband(0.02 * MaxAngularRate).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private DriveMode driveMode;

    private final Drivetrain drivetrain = Robot.drivetrain;
    private final Elevator elevator = Robot.elevator;
    private final Manipulator manipulato = Robot.manipulator;
    private final Climber climber = Robot.climber;
    private final Limelight lineupCamera = Robot.limelight;
    private final Arm arm = Robot.arm;
    private final Intake intake = Robot.intake;

    private final Idle idle = new Idle();
    private final ScoreLeft scoreLeft = new ScoreLeft();
    private final ScoreRight scoreRight = new ScoreRight();

    private final ZeroElevator zeroElevator = new ZeroElevator();
    private final DoNothing doNothing = new DoNothing();

    public RobotMode() {
        this.setDefaultCommand(idle);
    }

    @Override
    public void periodic() {

        switch (driveMode) {
            case TeleopDrive:

                drivetrain.setControl(teleopDrive.withVelocityX(SwerveXSupplier.get() < 0 ? Math.pow(SwerveXSupplier.get() * MaxSpeed, 2) : -Math.pow(SwerveXSupplier.get() * MaxSpeed, 2))
                                            .withVelocityY(SwerveYSupplier.get() < 0 ? Math.pow(SwerveYSupplier.get() * MaxSpeed, 2) : -Math.pow(SwerveYSupplier.get() * MaxSpeed, 2))
                                            .withRotationalRate(-SwerveRotationSupplier.get() * MaxAngularRate)
                );

                break;

            case FieldCentric:

                drivetrain.setControl(fieldCentric.withVelocityX(SwerveXSupplier.get() * MaxSpeed)
                                            .withVelocityY(SwerveYSupplier.get() * MaxSpeed)
                                            .withRotationalRate(SwerveRotationSupplier.get() * MaxAngularRate)
                );
                
                break;
        
            case RobotCentric:

                drivetrain.setControl(robotCentric.withVelocityX(SwerveXSupplier.get() * MaxSpeed)
                                                .withVelocityY(SwerveYSupplier.get() * MaxSpeed)
                                                .withRotationalRate(SwerveRotationSupplier.get() * MaxAngularRate)
                );

                break;
            
            case Brake:

                break;

            case Auton:

                break;

            default:
                break;
        }

    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public void setSwerveControl(Supplier<Double> swerveXSupplier, Supplier<Double> swerveYSupplier, Supplier<Double> swerveRotationSupplier) {
        this.SwerveXSupplier = swerveXSupplier;
        this.SwerveYSupplier = swerveYSupplier;
        this.SwerveRotationSupplier = swerveRotationSupplier;
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
