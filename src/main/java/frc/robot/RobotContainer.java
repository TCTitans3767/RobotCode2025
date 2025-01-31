// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.Choreo;
import choreo.auto.AutoFactory;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Auton.Autos;
import frc.robot.Commands.AlignWithLeftReef;
import frc.robot.Commands.AlignWithRightReef;
import frc.robot.Commands.TeleopDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

    public final Drivetrain drivetrain = Robot.getDrivetrain();
    public final Limelight limelight = Robot.getLimelight();

    private Command alignWithRightReef = new AlignWithRightReef(limelight);
    private Command alignWithLeftReef = new AlignWithLeftReef(limelight);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    // private final Joystick joystick = new Joystick(0);

    private AutoFactory autoFactory;

    public RobotContainer() {
        configureBindings();
        configureChoreo();

        limelight.initialPoseEstimates();

        SmartDashboard.putData(drivetrain.getField());

    }

    private void configureChoreo() {
        autoFactory = new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetPose, 
            drivetrain::followTrajectory,
            true,
            drivetrain
        );
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(Math.pow(-joystick.getLeftY() * MaxSpeed, 3)) // Drive forward with negative Y (forward)
            //         .withVelocityY(Math.pow(-joystick.getLeftX() * MaxSpeed, 3)) // Drive left with negative X (left)
            //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            // )
            // new TeleopDrive(() -> drive.withVelocityX(Math.pow(-joystick.getRawAxis(1) * MaxSpeed, 3)) // Drive forward with negative Y (forward)
            //                             .withVelocityY(Math.pow(-joystick.getRawAxis(0) * MaxSpeed, 3)) // Drive left with negative X (left)
            //                             .withRotationalRate(-joystick.getRawAxis(2) * MaxAngularRate), drivetrain) // Drive counterclockwise with negative X (left), drivetrain)
            new TeleopDrive(() -> drive.withVelocityX(joystick.getLeftY() < 0 ? Math.pow(-joystick.getLeftY() * MaxSpeed, 2) : -Math.pow(-joystick.getLeftY() * MaxSpeed, 2)) // Drive forward with negative Y (forward)
                                        .withVelocityY(joystick.getLeftX() < 0 ? Math.pow(-joystick.getLeftX() * MaxSpeed, 2) : -Math.pow(-joystick.getLeftX() * MaxSpeed, 2)) // Drive left with negative X (left)
                                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate), drivetrain) // Drive counterclockwise with negative X (left), drivetrain)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.rightBumper().onTrue(limelight.runOnce(() -> limelight.resetIMU(new Rotation3d())));
        joystick.povUp().onTrue(limelight.runOnce(() -> limelight.initialPoseEstimates()));
        joystick.b().whileTrue(alignWithRightReef);
        joystick.x().whileTrue(alignWithLeftReef);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Autos.test(autoFactory).cmd();
    }

}
