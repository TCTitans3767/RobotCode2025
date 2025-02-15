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
import frc.robot.Commands.DriveCommands.AlignWithCoralStation;
import frc.robot.Commands.DriveCommands.AlignWithLeftReef;
import frc.robot.Commands.DriveCommands.AlignWithRightReef;
import frc.robot.Commands.DriveCommands.ControllerDrive;
import frc.robot.Commands.StateCommands.CoralFloor;
import frc.robot.Commands.StateCommands.CoralFloorPose;
import frc.robot.Commands.StateCommands.CoralReef;
import frc.robot.Commands.StateCommands.CoralReefAlignPose;
import frc.robot.Commands.StateCommands.CoralReefAligned;
import frc.robot.Commands.StateCommands.CoralReefPose;
import frc.robot.Commands.StateCommands.CoralScorePose;
import frc.robot.Commands.StateCommands.CoralStation;
import frc.robot.Commands.StateCommands.CoralStationAlignPose;
import frc.robot.Commands.StateCommands.CoralStationPose;
import frc.robot.Commands.StateCommands.EjectCoralPose;
import frc.robot.Commands.StateCommands.Transit;
import frc.robot.Commands.StateCommands.TransitPose;
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

    public Command currentDriveMode = null;

    public Command currentMode = null;
    public Command previousPose = null;

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

    public final static AlignWithCoralStation alignWithCoralStation = new AlignWithCoralStation();
    public final static AlignWithLeftReef alignWithLeftReef = new AlignWithLeftReef();
    public final static AlignWithRightReef alignWithRightReef = new AlignWithRightReef();
    public final static ControllerDrive controllerDrive = new ControllerDrive();

    public final static CoralStation coralStation = new CoralStation();
    public final static TransitPose transitPose = new TransitPose();
    public final static Transit transit = new Transit();
    public final static CoralStationPose coralStationPose = new CoralStationPose();
    public final static CoralStationAlignPose coralStationAlignPose = new CoralStationAlignPose();
    public final static CoralFloorPose coralFloorPose = new CoralFloorPose();
    public final static CoralFloor coralFloor = new CoralFloor();
    public final static CoralReefPose coralReefPose = new CoralReefPose();
    public final static CoralReef coralReef = new CoralReef();
    public final static CoralReefAlignPose coralReefAlignPose = new CoralReefAlignPose();
    public final static CoralReefAligned coralReefAligned = new CoralReefAligned();
    public final static CoralScorePose coralScorePose = new CoralScorePose();

    public final static EjectCoralPose ejectCoralPose = new EjectCoralPose();

    public RobotMode() {
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

    public void setDriveModeCommand(Command newDriveMode) {
        if (currentDriveMode != null) {
            currentDriveMode.cancel();
        }
        currentDriveMode = newDriveMode;
        currentDriveMode.schedule();
    }

    public void setCurrentMode(Command newMode) {
        if (currentMode != null) {
            currentMode.cancel();
        }
        currentMode = newMode;
        currentMode.schedule();
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public void setSwerveControl(Supplier<Double> swerveXSupplier, Supplier<Double> swerveYSupplier, Supplier<Double> swerveRotationSupplier) {
        this.SwerveXSupplier = swerveXSupplier;
        this.SwerveYSupplier = swerveYSupplier;
        this.SwerveRotationSupplier = swerveRotationSupplier;
    }

    public void cancelAll() {
        CommandScheduler.getInstance().cancelAll();
    }
    
}
