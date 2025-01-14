package frc.robot.subsystems;

import java.io.IOException;
import java.lang.reflect.Array;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AprilTagCamera extends SubsystemBase{

    private static boolean doEstimationAll = true;
    private boolean doEstimation = true;
    private static Drivetrain drivetrain = RobotContainer.getDrivetrain();

    private static AprilTagFieldLayout fieldLayout;
    private final PhotonCamera camera;
    private final String cameraName;
    private final Transform3d robotToCam;
    private final PhotonPoseEstimator poseEstimator;

    public AprilTagCamera(String cameraName, Transform3d robotToCam) {
        try {
            fieldLayout = new AprilTagFieldLayout(Path.of("src\\main\\deploy\\ApriltagJSON\\2025-reefscape.json"));
        } catch(IOException e) {
            System.out.println(e);
        }

        this.cameraName = cameraName;
        this.camera = new PhotonCamera(cameraName);
        this.robotToCam = robotToCam;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    }

    @Override
    public void periodic() {
        if (doEstimationAll && doEstimation) {
            
            var results = camera.getAllUnreadResults();
            if (!results.isEmpty()) {
                for (PhotonPipelineResult result : results) {
                    if (result.getMultiTagResult().isPresent()) {
                        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
                        if (estimatedPose.isPresent()) {
                            drivetrain.addVisionMeasurement(new Pose2d(estimatedPose.get().estimatedPose.toPose2d().getTranslation(), drivetrain.getRotation3d().toRotation2d()));
                        }
                    }
                }
            }
        }
    }

    public void turnOff() {
        this.doEstimation = false;
    }

    public void turnOn() {
        this.doEstimation = true;
    }

    public static void turnOffAll() {
        AprilTagCamera.doEstimationAll = false;
    }

    public static void turnOnAll() {
        AprilTagCamera.doEstimationAll = true;
    }

    public static double getTagAngle(int tagID) {
        return fieldLayout.getTagPose(tagID).get().toPose2d().getRotation().getDegrees();
    }

    public static Pose2d getTagPose(int tagID) {
        return fieldLayout.getTagPose(tagID).get().toPose2d();
    }

    public static double getNearestReefAngle() {
        if (Robot.getAlliance() == Alliance.Blue) {
            List<Integer> nearestTags = Arrays.asList(Constants.ReefTagIDs.blue);
        }
    }
}
