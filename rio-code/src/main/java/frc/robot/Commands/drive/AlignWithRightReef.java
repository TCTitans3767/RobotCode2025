package frc.robot.Commands.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.DashboardButtonBox;
import frc.robot.Constants.ReefTagIDs;
import frc.robot.ButtonBox;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;
import frc.robot.utils.Logger;
import frc.robot.subsystems.Drivetrain;

public class AlignWithRightReef extends Command{
    
    private final Limelight camera = Robot.limelight;
    private final Drivetrain drivetrain = Robot.getDrivetrain();

    private final RobotCentric driveWithTag = new RobotCentric();
    private final FieldCentric driveWithOdometry = new FieldCentric();
    private final Pigeon2 rotation = drivetrain.getPigeon2();

    private final PIDController xController = new PIDController(Constants.Drive.YAlignementPIDkP, Constants.Drive.YAlignementPIDkI, Constants.Drive.YAlignementPIDkD);
    private final PIDController yController = new PIDController(Constants.Drive.YAlignementPIDkP, Constants.Drive.YAlignementPIDkI, Constants.Drive.YAlignementPIDkD);
    private final PIDController headingController = new PIDController(Constants.Drive.rotationAlignementPIDkP, Constants.Drive.rotationAlignementPIDkI, Constants.Drive.rotationAlignementPIDkD);

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;

    private int targetReefTag;
    private Rotation2d targetReefRotation;
    private Pose2d targetReefPose = new Pose2d();
    private Pose2d targetPose = new Pose2d();
    private final Field2d field = drivetrain.getField();
    
    public AlignWithRightReef() {
        headingController.enableContinuousInput(-180, 180);

        addRequirements(Robot.drivetrain);
    }

    @Override
    public void initialize() {
        Robot.lights.setBackLEDColor(LEDPattern.solid(Color.kOrange));
        switch (DashboardButtonBox.getSelectedReefBranch()) {
            case B:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefAB) : Limelight.getTagPose(ReefTagIDs.redReefAB);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(0)) : new Rotation2d(Units.degreesToRadians(180));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefAB : ReefTagIDs.redReefAB;
                break;
        
            case D:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefCD) : Limelight.getTagPose(ReefTagIDs.redReefCD);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(60)) : new Rotation2d(Units.degreesToRadians(-120));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefCD : ReefTagIDs.redReefCD;
                break;

            case F:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefEF) : Limelight.getTagPose(ReefTagIDs.redReefEF);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(120)) : new Rotation2d(Units.degreesToRadians(-60));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefEF : ReefTagIDs.redReefEF;
                break;

            case H:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefGH) : Limelight.getTagPose(ReefTagIDs.redReefGH);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(180)) : new Rotation2d(Units.degreesToRadians(0));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefGH : ReefTagIDs.redReefGH;
                break;

            case J:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefIJ) : Limelight.getTagPose(ReefTagIDs.redReefIJ);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(-120)) : new Rotation2d(Units.degreesToRadians(60));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefIJ : ReefTagIDs.redReefIJ;
                break;

            case L:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefKL) : Limelight.getTagPose(ReefTagIDs.redReefKL);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(-60)) : new Rotation2d(Units.degreesToRadians(120));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefKL : ReefTagIDs.redReefKL;
                break;

            default:
                Logger.logSystemError("AlignWithRightReef: Invalid branch: " + DashboardButtonBox.getSelectedReefBranch());
                this.cancel();
                break;
        }

        
        xController.setPID(Constants.Drive.XAlignementPIDkP, Constants.Drive.XAlignementPIDkI, Constants.Drive.XAlignementPIDkD);
        yController.setPID(Constants.Drive.YAlignementPIDkP, Constants.Drive.YAlignementPIDkI, Constants.Drive.YAlignementPIDkD);
        headingController.setPID(Constants.Drive.rotationAlignementPIDkP, Constants.Drive.rotationAlignementPIDkI, Constants.Drive.rotationAlignementPIDkD);

        targetPose = new Pose2d(targetReefPose.transformBy(new Transform2d((Constants.Robot.chassisDepthMeters/2), 0.1, new Rotation2d())).getTranslation(), targetReefRotation);

        Logger.log("Target Pose", targetPose.toString());

        headingController.setTolerance(Constants.Drive.headingAlignmentTolerance);
        headingController.setSetpoint(targetPose.getRotation().getDegrees());
        headingController.reset();

        xController.setTolerance(Constants.Drive.XAlignmentTolerance);
        yController.setTolerance(Constants.Drive.YAlignmentTolerance);

        camera.setTagFilter(new int[]{targetReefTag});

        Robot.robotMode.setDriveMode(DriveMode.FieldCentric);
        Robot.robotMode.setSwerveControl(() -> xVelocity, () -> yVelocity, () -> rotationVelocity);

    }

    @Override
    public void execute() {

        xVelocity = xController.calculate(drivetrain.getPose().getX(), targetPose.getX()) + (xController.getError() < 0 ? -Constants.Drive.XFeedForward : Constants.Drive.XFeedForward);
        yVelocity = yController.calculate(drivetrain.getPose().getY(), targetPose.getY()) + (yController.getError() < 0 ? -Constants.Drive.YFeedForward : Constants.Drive.YFeedForward);
        rotationVelocity = headingController.calculate(drivetrain.getPose().getRotation().getDegrees()) + (headingController.getError() < 0 ? Constants.Drive.rotationalFeedForward : -Constants.Drive.rotationalFeedForward);

        Logger.log("Alignment/Is Aligned", isAligned());

    }

    @Override
    public boolean isFinished() {
        return isAligned();
    }


    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);
        Robot.lights.setBackLEDColor(LEDPattern.solid(Color.kBlue));
        camera.resetTagFilter();
    }

    public boolean isAligned() {
        Logger.log("Alignment/Distance X", xController.getError());
        Logger.log("Alignment/Distance Y", yController.getError());
        return xController.atSetpoint() && yController.atSetpoint();
    }

}
