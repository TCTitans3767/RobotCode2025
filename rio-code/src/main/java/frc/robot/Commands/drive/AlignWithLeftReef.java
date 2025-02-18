package frc.robot.Commands.drive;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ReefTagIDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotMode.DriveMode;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.CommandTrigger;
import frc.robot.utils.Utils;

public class AlignWithLeftReef extends Command{
    
    private final Limelight camera = Robot.limelight;
    private final Drivetrain drivetrain = Robot.drivetrain;

    private final RobotCentric driveWithTag = new RobotCentric();
    private final FieldCentric driveWithOdometry = new FieldCentric();
    private final Pigeon2 rotation = drivetrain.getPigeon2();

    private final PIDController xController = new PIDController(1.6, 0, 0);
    private final PIDController yController = new PIDController(1.6, 0, 0);
    private final PIDController headingController = new PIDController(0.11, 0, 0);

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;

    private int targetReefTag;
    private Pose2d targetReefPose;
    private Pose2d odometryTargetPose;
    private final Field2d field = drivetrain.getField();
    
    public AlignWithLeftReef() {
        headingController.enableContinuousInput(-180, 180);

        addRequirements(Robot.drivetrain);
    }

    @Override
    public void initialize() {

        switch (Utils.getSelectedReefPosition()) {
            case A:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefAB) : Limelight.getTagPose(ReefTagIDs.redReefAB);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefAB : ReefTagIDs.redReefAB;
                break;
        
            case C:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefCD) : Limelight.getTagPose(ReefTagIDs.redReefCD);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefCD : ReefTagIDs.redReefCD;
                break;

            case E:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefEF) : Limelight.getTagPose(ReefTagIDs.redReefEF);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefEF : ReefTagIDs.redReefEF;
                break;

            case G:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefGH) : Limelight.getTagPose(ReefTagIDs.redReefGH);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefGH : ReefTagIDs.redReefGH;
                break;

            case I:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefIJ) : Limelight.getTagPose(ReefTagIDs.redReefIJ);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefIJ : ReefTagIDs.redReefIJ;
                break;

            case K:
                targetReefPose = Robot.getAlliance() == Alliance.Blue ? Limelight.getTagPose(ReefTagIDs.blueReefKL) : Limelight.getTagPose(ReefTagIDs.redReefKL);
                targetReefTag = Robot.getAlliance() == Alliance.Blue ? ReefTagIDs.blueReefKL : ReefTagIDs.redReefKL;
                break;
        }

        odometryTargetPose = targetReefPose.transformBy(new Transform2d((Constants.Robot.chassisDepthMeters/2), Units.inchesToMeters(-9), new Rotation2d(0)));

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
            yVelocity = yController.calculate(camera.getXFromTag(), Units.inchesToMeters(12));
            rotationVelocity = headingController.calculate(rotation.getYaw().getValueAsDouble());
        } else {
            // drivetrain.setControl(driveWithOdometry.withRotationalRate(headingController.calculate(rotation.getYaw().getValueAsDouble()))
            //                         .withVelocityX(xController.calculate(drivetrain.getPose().getX(), OdometryTargetPose.getX()))
            //                         .withVelocityY(yController.calculate(drivetrain.getPose().getY(), OdometryTargetPose.getY()))                    
            // );
            xVelocity = xController.calculate(drivetrain.getPose().getX(), odometryTargetPose.getX());
            yVelocity = yController.calculate(drivetrain.getPose().getY(), odometryTargetPose.getY());
            rotationVelocity = headingController.calculate(rotation.getYaw().getValueAsDouble());
        }

        field.getObject("targetObject").setPose(odometryTargetPose);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        camera.resetTagFilter();
    }

    public boolean isAligned() {
        return this.isScheduled() ? xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint() : false;
    }
}
