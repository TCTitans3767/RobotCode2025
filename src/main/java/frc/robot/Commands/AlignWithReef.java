package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTagCamera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.CommandTrigger;

public class AlignWithReef extends Command{
    
    private final AprilTagCamera camera;
    private final Drivetrain drivetrain = RobotContainer.getDrivetrain();
    private final BooleanSupplier rightBranch;

    private int nearestReefTag;
    private Pose2d nearestReefPose;

    private final PIDController xController = new PIDController(10, 0, 0);
    private final PIDController yController = new PIDController(10, 0, 0);
    private final PIDController headingController = new PIDController(7.5, 0, 0);
    
    public AlignWithReef(AprilTagCamera camera, BooleanSupplier rightBranch) {
        this.camera = camera;
        this.rightBranch = rightBranch;

        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        camera.turnOn();

        nearestReefTag = AprilTagCamera.getNearestReefTag();

        nearestReefPose = AprilTagCamera.getTagPose(nearestReefTag);

        headingController.setTolerance(0.5);
        headingController.setSetpoint(AprilTagCamera.getTagAngle(nearestReefTag).plus(Rotation2d.k180deg).getDegrees());
        headingController.reset();

        xController.setSetpoint(nearestReefPose.getX());

        yController.setSetpoint(nearestReefPose.getY());

    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
