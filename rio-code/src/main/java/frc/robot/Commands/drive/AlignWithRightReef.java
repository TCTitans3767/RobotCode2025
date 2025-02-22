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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ReefTagIDs;
import frc.robot.ButtonBox;
import frc.robot.Robot;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotMode.DriveMode;
import frc.robot.utils.Logger;
import frc.robot.subsystems.Drivetrain;

public class AlignWithRightReef extends Command{
    
    private final Limelight camera = Robot.limelight;
    private final Drivetrain drivetrain = Robot.getDrivetrain();

    private final RobotCentric driveWithTag = new RobotCentric();
    private final FieldCentric driveWithOdometry = new FieldCentric();
    private final Pigeon2 rotation = drivetrain.getPigeon2();

    private final PIDController xController = new PIDController(Constants.Drive.translationAlignementPIDkP, Constants.Drive.translationAlignementPIDkI, Constants.Drive.translationAlignementPIDkD);
    private final PIDController yController = new PIDController(Constants.Drive.translationAlignementPIDkP, Constants.Drive.translationAlignementPIDkI, Constants.Drive.translationAlignementPIDkD);
    private final PIDController headingController = new PIDController(Constants.Drive.rotationAlignementPIDkP, Constants.Drive.rotationAlignementPIDkI, Constants.Drive.rotationAlignementPIDkD);

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;

    private int targetReefTag;
    private Pose2d targetReefPose = new Pose2d();
    private Pose2d OdometryTargetPose = new Pose2d();
    private final Field2d field = drivetrain.getField();
    
    public AlignWithRightReef() {
        headingController.enableContinuousInput(-180, 180);

        addRequirements(Robot.drivetrain);
    }

    @Override
    public void initialize() {
        switch (ButtonBox.getSelectedBranch()) {
            case B:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefAB) : Limelight.getTagPose(ReefTagIDs.redReefAB);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefAB : ReefTagIDs.redReefAB;
                break;
        
            case D:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefCD) : Limelight.getTagPose(ReefTagIDs.redReefCD);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefCD : ReefTagIDs.redReefCD;
                break;

            case F:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefEF) : Limelight.getTagPose(ReefTagIDs.redReefEF);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefEF : ReefTagIDs.redReefEF;
                break;

            case H:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefGH) : Limelight.getTagPose(ReefTagIDs.redReefGH);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefGH : ReefTagIDs.redReefGH;
                break;

            case J:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefIJ) : Limelight.getTagPose(ReefTagIDs.redReefIJ);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefIJ : ReefTagIDs.redReefIJ;
                break;

            case L:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefKL) : Limelight.getTagPose(ReefTagIDs.redReefKL);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefKL : ReefTagIDs.redReefKL;
                break;
                
            default:
                Logger.logSystemError("AlignWithRightReef: Invalid branch: " + ButtonBox.getSelectedBranch());
                this.cancel();
                break;
        }

        OdometryTargetPose = targetReefPose.transformBy(new Transform2d((Constants.Robot.chassisDepthMeters/2), Units.inchesToMeters(9), new Rotation2d(0)));

        headingController.setTolerance(0.2);
        headingController.setSetpoint(targetReefPose.getRotation().plus(Rotation2d.k180deg).getDegrees());
        headingController.reset();

        xController.setTolerance(Constants.Drive.alignmentTolerance);
        yController.setTolerance(Constants.Drive.alignmentTolerance); 

        camera.setTagFilter(new int[]{targetReefTag});

        Robot.robotMode.setDriveMode(DriveMode.RobotCentric);
        Robot.robotMode.setSwerveControl(() -> xVelocity, () -> yVelocity, () -> rotationVelocity);

    }

    @Override
    public void execute() {

        if (camera.tagIsVisible()) {
            // drivetrain.setControl(driveWithTag.withRotationalRate(headingController.calculate(rotation.getYaw().getValueAsDouble()))
            //                         .withVelocityX(-xController.calculate(camera.getZFromTag(), (Constants.Robot.chassisDepthMeters/2)))
            //                         .withVelocityY(yController.calculate(camera.getXFromTag(), Units.inchesToMeters(-9)))
            //                     );
            xVelocity = -xController.calculate(camera.getZFromTag(), (Constants.Robot.chassisDepthMeters/2));
            yVelocity = yController.calculate(camera.getXFromTag(), Units.inchesToMeters(-9));
            rotationVelocity = headingController.calculate(rotation.getYaw().getValueAsDouble());
        } else {
            // drivetrain.setControl(driveWithOdometry.withRotationalRate(headingController.calculate(rotation.getYaw().getValueAsDouble()))
            //                         .withVelocityX(xController.calculate(drivetrain.getPose().getX(), OdometryTargetPose.getX()))
            //                         .withVelocityY(yController.calculate(drivetrain.getPose().getY(), OdometryTargetPose.getY()))                    
            // );
            xVelocity = xController.calculate(drivetrain.getPose().getX(), OdometryTargetPose.getX());
            yVelocity = yController.calculate(drivetrain.getPose().getY(), OdometryTargetPose.getY());
            rotationVelocity = headingController.calculate(rotation.getYaw().getValueAsDouble());
        }

        field.getObject("targetObject").setPose(OdometryTargetPose);
        
    }

    @Override
    public boolean isFinished() {
        return isAligned();
    }

    @Override
    public void end(boolean interrupted) {
        camera.resetTagFilter();
    }

    public boolean isAligned() {
        return xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint();
    }

}
