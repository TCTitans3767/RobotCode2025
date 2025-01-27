package frc.robot.subsystems;

import java.util.Vector;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    }

    @Override
    public void periodic() {

        if (!doEstimation || !doEstimationAll) {
            return;
        }

        PoseEstimate estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 720) {
            return;
        }
        if (estimatedPose.tagCount == 0) {
            return;
        }
        drivetrain.addVisionMeasurement(estimatedPose.pose, estimatedPose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 999999));

    }

    public void resetIMU(Rotation3d robotOrientation) {
        doEstimation = false;

        LimelightHelpers.SetIMUMode(limelightName, 1);
        LimelightHelpers.SetRobotOrientation(limelightName, 
            robotOrientation.getZ(),
            0,
            robotOrientation.getY(),
            0,
            robotOrientation.getX(),
            0
        );
        LimelightHelpers.SetIMUMode(limelightName, 2);

        doEstimation = true;
    }

    public void initialPoseEstimates() {
        while (startupEstimations < 10) {
            PoseEstimate result = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
            if (result.tagCount == 1) {
                if (result.rawFiducials[0].ambiguity > 0.3) {
                    continue;
                }
                if (result.rawFiducials[0].distToCamera > 3) {
                    continue;
                }
            }

            if (result.tagCount == 0) {
                continue;
            }

            drivetrain.addVisionMeasurement(result.pose, result.timestampSeconds, VecBuilder.fill(.25, .25, Units.degreesToRadians(0.5)));
            
        }

        resetIMU(drivetrain.getRotation3d());
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
    
}
