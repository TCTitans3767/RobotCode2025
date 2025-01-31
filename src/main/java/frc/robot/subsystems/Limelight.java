package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Vector;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightResults;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase{

    private final String limelightName;

    private final Drivetrain drivetrain = Robot.getDrivetrain();

    private boolean doEstimation = false;
    private static boolean doEstimationAll = true;
    private boolean doEstimationFrame = true;
    private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    private int startupEstimations = 0;
    
    public Limelight(String limelightName, Pose3d robotToLimelight) {
        this.limelightName = limelightName;

        LimelightHelpers.setCameraPose_RobotSpace(limelightName, 
            robotToLimelight.getX(),
            robotToLimelight.getY(), 
            robotToLimelight.getZ(), 
            robotToLimelight.getRotation().getX(), 
            robotToLimelight.getRotation().getY(), 
            robotToLimelight.getRotation().getZ()
        );

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.3, 0.3, 999999));

    }

    @Override
    public void periodic() {
        doEstimationFrame = true;

        PoseEstimate estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 720) {
            doEstimationFrame = false;
        }
        if (estimatedPose.tagCount == 0) {
            doEstimationFrame = false;
        }

        if (doEstimationFrame && doEstimation && doEstimationAll) {
            drivetrain.addVisionMeasurement(estimatedPose.pose);
        }

        SmartDashboard.putNumber(limelightName + "/Target Pose X", LimelightHelpers.getTargetPose_RobotSpace(limelightName)[0]);
        SmartDashboard.putNumber(limelightName + "/Target Pose Y", LimelightHelpers.getTargetPose_RobotSpace(limelightName)[1]);
        SmartDashboard.putNumber(limelightName + "/Target Pose Z", LimelightHelpers.getTargetPose_RobotSpace(limelightName)[2]);

    }

    public void resetIMU(Rotation3d robotOrientation) {
        doEstimation = false;

        LimelightHelpers.SetIMUMode(limelightName, 1);
        LimelightHelpers.SetRobotOrientation(limelightName, 
            robotOrientation.getZ(),
            0,
            0,
            0,
            0,
            0
        );
        LimelightHelpers.SetIMUMode(limelightName, 2);

        doEstimation = true;
    }

    public void setTagFilter(int[] ids) {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, ids);
    }

    public void resetTagFilter() {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22});
    }

    public double getXFromTag() {
        return LimelightHelpers.getTargetPose_RobotSpace(limelightName)[0];
    }

    public double getYFromTag() {
        return LimelightHelpers.getTargetPose_RobotSpace(limelightName)[1];
    }

    public double getZFromTag() {
        return LimelightHelpers.getTargetPose_RobotSpace(limelightName)[2];
    }

    public boolean tagIsVisible() {
        return LimelightHelpers.getTV(limelightName);
    }

    public void initialPoseEstimates() {
        doEstimation = false;
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.5));
        int numTries = 0;
        while (startupEstimations < 10 && numTries < 40) {
            PoseEstimate result = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
            if (result.tagCount == 1) {
                if (result.rawFiducials[0].ambiguity > 0.3) {
                    numTries++;
                    continue;
                }
                if (result.rawFiducials[0].distToCamera > 3) {
                    numTries++;
                    continue;
                }
            }

            if (result.tagCount == 0) {
                numTries++;
                continue;
            }

            drivetrain.addVisionMeasurement(result.pose);
            
            startupEstimations++;
        }

        resetIMU(drivetrain.getRotation3d());
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.3, 0.3, 99999));
        turnOnAprilTags();
    }

    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }  

    public void turnOffAprilTags() {
        doEstimation = false;
    }

    public void turnOnAprilTags() {
        doEstimation = true;
    }

    public static void turnOffAllAprilTags() {
        doEstimationAll = false;
    }

    public static void turnOnAllAprilTags() {
        doEstimationAll = true;
    }

        public static Rotation2d getTagAngle(int tagID) {
        return fieldLayout.getTagPose(tagID).get().toPose2d().getRotation();
    }

    public static Pose2d getTagPose(int tagID) {
        return fieldLayout.getTagPose(tagID).get().toPose2d();
    }

    public static double getNearestReefAngle() {
        if (Robot.getAlliance() == Alliance.Blue) {
            List<Integer> nearestTags = Arrays.asList(Constants.ReefTagIDs.blue);

            nearestTags.sort(new Comparator<Integer>() {

                Drivetrain drivetrain = Robot.getDrivetrain();

                @Override
                public int compare(Integer tag1, Integer tag2) {
                    double distanceToTag1 = Limelight.getTagPose(tag1).getTranslation().getDistance(drivetrain.getPose().getTranslation());
                    double distanceToTag2 = Limelight.getTagPose(tag2).getTranslation().getDistance(drivetrain.getPose().getTranslation());

                    return (int) Math.floor(distanceToTag1 - distanceToTag2);
                }
            });

            return getTagAngle(nearestTags.get(0)).plus(Rotation2d.fromDegrees(180)).getDegrees();

        } else {
            List<Integer> nearestTags = Arrays.asList(Constants.ReefTagIDs.red);

            nearestTags.sort(new Comparator<Integer>() {

                Drivetrain drivetrain = Robot.getDrivetrain();

                @Override
                public int compare(Integer tag1, Integer tag2) {
                    double distanceToTag1 = Limelight.getTagPose(tag1).getTranslation().getDistance(drivetrain.getPose().getTranslation());
                    double distanceToTag2 = Limelight.getTagPose(tag2).getTranslation().getDistance(drivetrain.getPose().getTranslation());

                    return (int) Math.floor(distanceToTag1 - distanceToTag2);
                }
            });

            return getTagAngle(nearestTags.get(0)).plus(Rotation2d.fromDegrees(180)).getDegrees();
        }
    }

    public static int getNearestReefTag() {
        if (Robot.getAlliance() == Alliance.Blue) {
            List<Integer> nearestTags = Arrays.asList(Constants.ReefTagIDs.blue);

            nearestTags.sort(new Comparator<Integer>() {

                Drivetrain drivetrain = Robot.getDrivetrain();

                @Override
                public int compare(Integer tag1, Integer tag2) {
                    double distanceToTag1 = Limelight.getTagPose(tag1).getTranslation().getDistance(drivetrain.getPose().getTranslation());
                    double distanceToTag2 = Limelight.getTagPose(tag2).getTranslation().getDistance(drivetrain.getPose().getTranslation());

                    return (int) Math.floor(distanceToTag1 - distanceToTag2);
                }
            });

            return nearestTags.get(0);

        } else {
            List<Integer> nearestTags = Arrays.asList(Constants.ReefTagIDs.red);

            nearestTags.sort(new Comparator<Integer>() {

                Drivetrain drivetrain = Robot.getDrivetrain();

                @Override
                public int compare(Integer tag1, Integer tag2) {
                    double distanceToTag1 = Limelight.getTagPose(tag1).getTranslation().getDistance(drivetrain.getPose().getTranslation());
                    double distanceToTag2 = Limelight.getTagPose(tag2).getTranslation().getDistance(drivetrain.getPose().getTranslation());

                    return (int) Math.floor(distanceToTag1 - distanceToTag2);
                }
            });

            return nearestTags.get(0);
        }
    }
    
}
