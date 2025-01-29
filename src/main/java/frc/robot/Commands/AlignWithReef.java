package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.CommandTrigger;

public class AlignWithReef extends Command{
    
    private final Limelight camera;
    private final Drivetrain drivetrain = Robot.getDrivetrain();
    private final BooleanSupplier rightBranch;

    private final RobotCentric drive = new RobotCentric();
    private final Pigeon2 rotation = drivetrain.getPigeon2();

    private int nearestReefTag;
    private Pose2d nearestReefPose;

    private final PIDController xController = new PIDController(1, 0, 0);
    private final PIDController yController = new PIDController(1, 0, 0);
    private final PIDController headingController = new PIDController(0.1, 0, 0);
    
    public AlignWithReef(Limelight camera, BooleanSupplier rightBranch) {
        this.camera = camera;
        this.rightBranch = rightBranch;

        headingController.enableContinuousInput(-180, 180);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

        nearestReefTag = Limelight.getNearestReefTag();

        nearestReefPose = Limelight.getTagPose(nearestReefTag);

        headingController.setTolerance(0.2);
        headingController.setSetpoint(Limelight.getTagAngle(nearestReefTag).plus(Rotation2d.k180deg).getDegrees());
        headingController.reset();

        camera.setTagFilter(new int[]{nearestReefTag});

    }

    @Override
    public void execute() {

        if (camera.tagIsVisible()) {
            drivetrain.setControl(drive.withRotationalRate(headingController.calculate(rotation.getYaw().getValueAsDouble()))
                                    .withVelocityX(xController.calculate(camera.getXFromTag(), .5))
                                    .withVelocityY(yController.calculate(camera.getYFromTag(), 0))
                                );
        } else {
            drivetrain.setControl(drive.withRotationalRate(headingController.calculate(rotation.getYaw().getValueAsDouble())));
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        camera.resetTagFilter();
    }

}
