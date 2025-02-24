package frc.robot.Commands.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ButtonBox;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RobotMode.DriveMode;

public class AlignWithCoralStation extends Command{

    private final Drivetrain drivetrain = Robot.drivetrain;

    private final PIDController xController = new PIDController(Constants.Drive.YAlignementPIDkP, Constants.Drive.YAlignementPIDkI, Constants.Drive.YAlignementPIDkD);
    private final PIDController yController = new PIDController(Constants.Drive.YAlignementPIDkP, Constants.Drive.YAlignementPIDkI, Constants.Drive.YAlignementPIDkD);
    private final PIDController headingController = new PIDController(Constants.Drive.rotationAlignementPIDkP, Constants.Drive.rotationAlignementPIDkI, Constants.Drive.rotationAlignementPIDkD);

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;
    
    private Pose2d targetPose;

    private Pigeon2 pigeon = drivetrain.getPigeon2();
    
    public AlignWithCoralStation() {
        headingController.enableContinuousInput(-180, 180);
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void initialize() {
        if (Robot.getAlliance() == Alliance.Red) {
            if (ButtonBox.isCoralStationAlignRight()) {
                if (Robot.drivetrain.isRightStationCloser()) {
                    targetPose = Constants.Field.redRightCoralStationRightAlignment;
                } else {
                    targetPose = Constants.Field.redLeftCoralStationRightAlignment;
                }
            } else {
                if (Robot.drivetrain.isRightStationCloser()) {
                    targetPose = Constants.Field.redRightCoralStationLeftAlignment;
                } else {
                    targetPose = Constants.Field.redLeftCoralStationLeftAlignment;
                }
            }
        } else {
            if (ButtonBox.isCoralStationAlignRight()) {
                if (Robot.drivetrain.isRightStationCloser()) {
                    targetPose = Constants.Field.blueRightCoralStationRightAlignment;
                } else {
                    targetPose = Constants.Field.blueLeftCoralStationRightAlignment;
                }
            } else {
                if (Robot.drivetrain.isRightStationCloser()) {
                    targetPose = Constants.Field.blueRightCoralStationLeftAlignment;
                } else {
                    targetPose = Constants.Field.blueLeftCoralStationLeftAlignment;
                }
            }
        }

        headingController.reset();
        headingController.setSetpoint(targetPose.getRotation().getDegrees());
        headingController.setTolerance(0.1);

        xController.reset();
        xController.setSetpoint(targetPose.getTranslation().getX());
        xController.setTolerance(Constants.Drive.headingAlignmentTolerance);

        yController.reset();
        yController.setSetpoint(targetPose.getTranslation().getY());
        yController.setTolerance(Constants.Drive.headingAlignmentTolerance);

        Robot.robotMode.setDriveMode(DriveMode.RobotCentric);
        Robot.robotMode.setSwerveControl(() -> xVelocity, () -> yVelocity, () -> rotationVelocity);
    }

    @Override
    public void execute() {
        xVelocity = xController.calculate(drivetrain.getPose().getX());
        yVelocity = yController.calculate(drivetrain.getPose().getY());
        rotationVelocity = headingController.calculate(pigeon.getYaw().getValueAsDouble());
    }

    @Override
    public boolean isFinished() {
        return isAligned();
    }

    public boolean isAligned() {
        return xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint();
    }

}
