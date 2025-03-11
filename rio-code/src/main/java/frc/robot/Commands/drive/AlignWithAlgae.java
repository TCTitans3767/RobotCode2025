package frc.robot.Commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ReefTagIDs;
import frc.robot.DashboardButtonBox;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.poses.CoralReefAlignPose;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;
import frc.robot.utils.Logger;

public class AlignWithAlgae extends Command{

    private final PIDController xController = new PIDController(Constants.Drive.XAlignementPIDkP, Constants.Drive.XAlignementPIDkI, Constants.Drive.XAlignementPIDkD);
    private final PIDController yController = new PIDController(Constants.Drive.YAlignementPIDkP, Constants.Drive.YAlignementPIDkI, Constants.Drive.YAlignementPIDkD);
    private final PIDController headingController = new PIDController(Constants.Drive.rotationAlignementPIDkP, Constants.Drive.rotationAlignementPIDkI, Constants.Drive.rotationAlignementPIDkD);

    private Pose2d initialPosition;
    private Pose2d targetPose;
    private Pose2d targetReefPose;
    private Rotation2d targetReefRotation;
    private int targetReefTag;

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;
    
    public AlignWithAlgae() {

        headingController.enableContinuousInput(-180, 180);
        xController.setTolerance(Constants.Drive.XAlignmentTolerance);
        yController.setTolerance(Constants.Drive.YAlignmentTolerance);
        headingController.setTolerance(Constants.Drive.headingAlignmentTolerance);

        addRequirements(Robot.drivetrain);
    }

    @Override
    public void initialize() {

        switch (DashboardButtonBox.getSelectedReefBranch()) {
            case A:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefAB) : Limelight.getTagPose(ReefTagIDs.redReefAB);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(0)) : new Rotation2d(Units.degreesToRadians(180));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefAB : ReefTagIDs.redReefAB;
                break;
            
            case C:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefCD) : Limelight.getTagPose(ReefTagIDs.redReefCD);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(60)) : new Rotation2d(Units.degreesToRadians(-120));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefCD : ReefTagIDs.redReefCD;
                break;

            case E:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefEF) : Limelight.getTagPose(ReefTagIDs.redReefEF);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(120)) : new Rotation2d(Units.degreesToRadians(-60));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefEF : ReefTagIDs.redReefEF;
                break;

            case G:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefGH) : Limelight.getTagPose(ReefTagIDs.redReefGH);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(180)) : new Rotation2d(Units.degreesToRadians(0));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefGH : ReefTagIDs.redReefGH;
                break;

            case I:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefIJ) : Limelight.getTagPose(ReefTagIDs.redReefIJ);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(-120)) : new Rotation2d(Units.degreesToRadians(60));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefIJ : ReefTagIDs.redReefIJ;
                break;

            case K:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefKL) : Limelight.getTagPose(ReefTagIDs.redReefKL);
                targetReefRotation = Robot.getAlliance() == Alliance.Blue ? new Rotation2d(Units.degreesToRadians(-60)) : new Rotation2d(Units.degreesToRadians(120));
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefKL : ReefTagIDs.redReefKL;
                break;

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
                Logger.logSystemError("AlignWithLeftReef: Invalid branch: " + DashboardButtonBox.getSelectedReefBranch());
                this.cancel();
                break;
        }

        initialPosition = Robot.drivetrain.getPose();
        targetPose = new Pose2d(targetReefPose.transformBy(new Transform2d((Constants.Robot.chassisDepthMeters/2), 0, new Rotation2d())).getTranslation(), targetReefRotation);
        Logger.log("Alignment/Target Pose", targetPose.toString());
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        headingController.setSetpoint(targetPose.getRotation().getDegrees());

        Robot.robotMode.setDriveMode(DriveMode.FieldCentric);
        Robot.robotMode.setSwerveControl(() -> xVelocity, () -> yVelocity, () -> rotationVelocity);
    }

    @Override
    public void execute() {
        xVelocity = xController.calculate(Robot.drivetrain.getPose().getX(), targetReefPose.getX()) + (xController.getError() < 0 ? Constants.Drive.XFeedForward : -Constants.Drive.XFeedForward);
        yVelocity = yController.calculate(Robot.drivetrain.getPose().getY(), targetReefPose.getY()) + (yController.getError() < 0 ? Constants.Drive.YFeedForward : -Constants.Drive.YFeedForward);
        rotationVelocity = headingController.calculate(Robot.drivetrain.getPose().getRotation().getDegrees()) + (headingController.getError() < 0 ? Constants.Drive.rotationalFeedForward : -Constants.Drive.rotationalFeedForward);

        Logger.log("Alignment/Is Aligned", isAligned());
    }

    @Override
    public boolean isFinished() {
        return isAligned();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);
    }

    public boolean isAligned() {
        Logger.log("Alignment/Distance X", xController.getError());
        Logger.log("Alignment/Distance Y", yController.getError());
        return xController.atSetpoint() && yController.atSetpoint();
    }

}
