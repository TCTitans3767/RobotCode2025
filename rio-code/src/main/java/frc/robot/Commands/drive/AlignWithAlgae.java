package frc.robot.Commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.poses.CoralReefAlignPose;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RobotMode;
import frc.robot.subsystems.RobotMode.DriveMode;
import frc.robot.utils.Logger;

public class AlignWithAlgae extends Command{

    private final PIDController xController = new PIDController(Constants.Drive.XAlignementPIDkP, Constants.Drive.XAlignementPIDkI, Constants.Drive.XAlignementPIDkD);
    private final PIDController yController = new PIDController(Constants.Drive.YAlignementPIDkP, Constants.Drive.YAlignementPIDkI, Constants.Drive.YAlignementPIDkD);
    private final PIDController headingController = new PIDController(Constants.Drive.rotationAlignementPIDkP, Constants.Drive.rotationAlignementPIDkI, Constants.Drive.rotationAlignementPIDkD);

    private Pose2d initialPosition;
    private Pose2d targetPose;

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
        initialPosition = Robot.drivetrain.getPose();
        targetPose = initialPosition.transformBy(CoralReefAlignPose.isLeftBranchSelected() ? new Transform2d(-0.05, -0.17, new Rotation2d()) : new Transform2d(0, 0.17, new Rotation2d()));
        Logger.log("Alignment/Target Pose", targetPose.toString());
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        headingController.setSetpoint(targetPose.getRotation().getDegrees());

        Robot.robotMode.setDriveMode(DriveMode.FieldCentric);
        Robot.robotMode.setSwerveControl(() -> xVelocity, () -> yVelocity, () -> rotationVelocity);
    }

    @Override
    public void execute() {
        xVelocity = xController.calculate(Robot.drivetrain.getPose().getX(), targetPose.getX()) + (xController.getError() < 0 ? Constants.Drive.XFeedForward : -Constants.Drive.XFeedForward);
        yVelocity = yController.calculate(Robot.drivetrain.getPose().getY(), targetPose.getY()) + (yController.getError() < 0 ? Constants.Drive.YFeedForward : -Constants.Drive.YFeedForward);
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
