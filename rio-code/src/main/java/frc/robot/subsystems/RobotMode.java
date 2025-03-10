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
import frc.robot.Commands.SetDriveModeCommand;
import frc.robot.Commands.SetModeCommand;
import frc.robot.Commands.drive.AlignWithAlgae;
import frc.robot.Commands.drive.AlignWithCoralStation;
import frc.robot.Commands.drive.AlignWithLeftReef;
import frc.robot.Commands.drive.AlignWithRightReef;
import frc.robot.Commands.drive.SlowControllerDrive;
import frc.robot.Commands.drive.ControllerDrive;
import frc.robot.Commands.elevator.ZeroElevator;
import frc.robot.Commands.modes.AlgaePickup;
import frc.robot.Commands.modes.Climb;
import frc.robot.Commands.modes.CoralFloor;
import frc.robot.Commands.modes.CoralReef;
import frc.robot.Commands.modes.CoralReefAligned;
import frc.robot.Commands.modes.CoralStation;
import frc.robot.Commands.modes.FinalClimb;
import frc.robot.Commands.modes.Transit;
import frc.robot.Commands.poses.AlgaePickupPose;
import frc.robot.Commands.poses.ClimbPose;
import frc.robot.Commands.poses.CoralFloorPose;
import frc.robot.Commands.poses.CoralRecievedPose;
import frc.robot.Commands.poses.CoralReefAlignPose;
import frc.robot.Commands.poses.CoralReefPose;
import frc.robot.Commands.poses.CoralStationAlignPose;
import frc.robot.Commands.poses.CoralStationPose;
import frc.robot.Commands.poses.DeployClimberPose;
import frc.robot.Commands.poses.EjectAlgaePose;
import frc.robot.Commands.poses.ScoreCoralPose;
import frc.robot.Commands.poses.InitialTransitPose;
import frc.robot.Commands.poses.KnockOffAlgaePose;
import frc.robot.Commands.poses.KnockOffAlgaePoseManual;
import frc.robot.Commands.poses.ResetPose;
import frc.robot.Commands.poses.TransitPose;
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
    private double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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

    public static AlignWithCoralStation alignWithCoralStation;
    public static AlignWithLeftReef alignWithLeftReef;
    public static AlignWithRightReef alignWithRightReef;
    public static ControllerDrive controllerDrive;

    public static CoralStation coralStation;
    public static TransitPose transitPose;
    public static Transit transit;
    public static CoralStationPose coralStationPose;
    public static CoralStationAlignPose coralStationAlignPose;
    public static CoralFloorPose coralFloorPose;
    public static CoralFloor coralFloor;
    public static CoralReefPose coralReefPose;
    public static CoralReef coralReef;
    public static CoralReefAlignPose coralReefAlignPose;
    public static CoralReefAligned coralReefAligned;
    public static ScoreCoralPose scoreCoralPose;
    public static AlgaePickupPose algaePickupPose;
    public static AlgaePickup algaePickup;
    public static InitialTransitPose initialTransitPose;
    public static CoralRecievedPose coralRecievedPose;
    public static AlignWithAlgae alignWithAlgae;
    public static Climb climb;
    public static FinalClimb finalClimb;
    public static DeployClimberPose deployClimberPose;
    public static ClimbPose climbPose;
    public static EjectAlgaePose ejectAlgaePose;

    public static SetDriveModeCommand setDriveModeCommand;
    public static SetModeCommand setModeCommand;

    public static KnockOffAlgaePose knockOffAlgaePose;
    public static KnockOffAlgaePoseManual knockOffAlgaePoseManual;

    public static SlowControllerDrive slowControllerDrive;

    public static ResetPose resetPose;

    public RobotMode() {
        SwerveXSupplier = () -> 0.0;
        SwerveYSupplier = () -> 0.0;
        SwerveRotationSupplier = () -> 0.0;

        alignWithCoralStation = new AlignWithCoralStation();
        alignWithLeftReef = new AlignWithLeftReef();
        alignWithRightReef = new AlignWithRightReef();
        controllerDrive = new ControllerDrive();
    
        coralStation = new CoralStation();
        transitPose = new TransitPose();
        transit = new Transit();
        coralStationPose = new CoralStationPose();
        coralStationAlignPose = new CoralStationAlignPose();
        coralFloorPose = new CoralFloorPose();
        coralFloor = new CoralFloor();
        coralReefPose = new CoralReefPose();
        coralReef = new CoralReef();
        coralReefAlignPose = new CoralReefAlignPose();
        coralReefAligned = new CoralReefAligned();
        scoreCoralPose = new ScoreCoralPose();
        algaePickupPose = new AlgaePickupPose();
        algaePickup = new AlgaePickup();
        initialTransitPose = new InitialTransitPose();
        coralRecievedPose = new CoralRecievedPose();
        alignWithAlgae = new AlignWithAlgae();
        knockOffAlgaePose = new KnockOffAlgaePose();
        climb = new Climb();
        finalClimb = new FinalClimb();
        deployClimberPose = new DeployClimberPose();
        climbPose = new ClimbPose();
        ejectAlgaePose = new EjectAlgaePose();
        
        knockOffAlgaePoseManual = new KnockOffAlgaePoseManual();

        slowControllerDrive = new SlowControllerDrive();

        resetPose = new ResetPose();
    }

    @Override
    public void periodic() {

        Logger.log("current mode", currentMode != null ? currentMode.getName() : "null");
        Logger.log("current drive command", currentDriveMode != null ? currentDriveMode.getName() : "null");
        Logger.log("Drivetrain/Is Near Reef", Robot.drivetrain.isNearToReef());
        Logger.log("Drivetrain/Current Yaw", Robot.drivetrain.getPose().getRotation().getDegrees());

        Logger.log("Is Current Pose Finsihed", currentMode != null ? currentMode.isFinished() : false);

        Logger.log("Is CoralFloor Scheduled", RobotMode.coralFloor.isScheduled());

        if (driveMode != null) {
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

    public boolean isDriveCommandFinished() {
        return currentDriveMode.isFinished();
    }
    
}
