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
import edu.wpi.first.math.estimator.SteadyStateKalmanFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.AutonCommands.PrepL4Auton;
import frc.robot.Commands.Intake.SetIntakePivotSpeed;
import frc.robot.Commands.arm.SetArmSpeed;
import frc.robot.Commands.climb.SetClimberSpeed;
import frc.robot.Commands.drive.AlignWithLeftReef;
import frc.robot.Commands.drive.AlignWithRightReef;
import frc.robot.Commands.drive.TeleopDrive;
import frc.robot.Commands.elevator.SetElevatorSpeed;
import frc.robot.Commands.poses.ScoreCoralPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.RobotMode;

public class RobotContainer {

    public final Drivetrain drivetrain = Robot.drivetrain;
    public final Limelight limelight = Robot.limelight;
    public final Elevator elevator = Robot.elevator;
    public final Manipulator manipulator = Robot.manipulator;
    public final Climber climber = Robot.climber;
    public final Arm arm = Robot.arm;
    public final Intake intake = Robot.intake;

    public final CommandXboxController joystick = Robot.joystick;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // private final Joystick joystick = new Joystick(0);

    public AutoFactory autoFactory;

    private Command rightL1;
    private Command leftL1;
    private Command rightL1NoExtras;

    public final static String rightL1Name = "Right L1";
    public final static String leftL1Name = "Left L1";
    public final static String rightL1NoExtrasName = "Right L1 No Extras"; 
    
    private final SendableChooser<Command> autonSelector = new SendableChooser<Command>();

    public RobotContainer() {

        configureBindings();
        configureChoreo();

        leftL1 = Autos.L1Left(autoFactory).cmd();
        rightL1 = Autos.L1Right(autoFactory).cmd();
        rightL1NoExtras = Autos.L1RightNoExtra(autoFactory).cmd();
    
        autonSelector.setDefaultOption(rightL1Name, rightL1);
        autonSelector.addOption(leftL1Name, leftL1);
        autonSelector.addOption(rightL1NoExtrasName, rightL1NoExtras);
        autonSelector.addOption(leftL1Name + " + L4", Autos.L1LeftCommandGroup(autoFactory));
        autonSelector.addOption("Left Wall Auton", Autos.lolipopAuto(autoFactory));
        SmartDashboard.putData("Auton Selection", autonSelector);

        // Robot.robotMode.setCurrentMode(RobotMode.initialTransitPose);
        // Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);

        limelight.initialPoseEstimates();

    }

    private void configureChoreo() {
        autoFactory = new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetPose, 
            drivetrain::followTrajectory,
            true,
            drivetrain
        );

        autoFactory.bind("PrepL4", new PrepL4Auton());
        autoFactory.bind("ScoreCoral", new ScoreCoralPose());
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     // drivetrain.applyRequest(() ->
        //     //     drive.withVelocityX(Math.pow(-joystick.getLeftY() * MaxSpeed, 3)) // Drive forward with negative Y (forward)
        //     //         .withVelocityY(Math.pow(-joystick.getLeftX() * MaxSpeed, 3)) // Drive left with negative X (left)
        //     //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     // )
        //     // new TeleopDrive(() -> drive.withVelocityX(Math.pow(-joystick.getRawAxis(1) * MaxSpeed, 3)) // Drive forward with negative Y (forward)
        //     //                             .withVelocityY(Math.pow(-joystick.getRawAxis(0) * MaxSpeed, 3)) // Drive left with negative X (left)
        //     //                             .withRotationalRate(-joystick.getRawAxis(2) * MaxAngularRate), drivetrain) // Drive counterclockwise with negative X (left), drivetrain)
        //     new TeleopDrive(() -> drive.withVelocityX(joystick.getLeftY() < 0 ? Math.pow(-joystick.getLeftY(), 3)  * MaxSpeed : -Math.pow(-joystick.getLeftY(), 3) * MaxSpeed) // Drive forward with negative Y (forward)
        //                                 .withVelocityY(joystick.getLeftX() < 0 ? Math.pow(-joystick.getLeftX(), 3) * MaxSpeed : -Math.pow(-joystick.getLeftX(), 3) * MaxSpeed) // Drive left with negative X (left)
        //                                 .withRotationalRate(-joystick.getRightX() * MaxAngularRate), drivetrain) // Drive counterclockwise with negative X (left), drivetrain)
        // );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
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
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.rightBumper().onTrue(limelight.runOnce(() -> limelight.resetIMU(new Rotation3d())));
        // joystick.povUp().onTrue(limelight.runOnce(() -> limelight.initialPoseEstimates()));
        // joystick.povLeft().whileTrue(new RunCommand(() -> {manipulator.setSpeed(0.1);}, manipulator));
        // joystick.povRight().whileTrue(new RunCommand(() -> {manipulator.setSpeed(-0.1);}, manipulator));
        // joystick.leftTrigger(0.05).onTrue(new InstantCommand(() -> Robot.getElevator().setSpeed(Math.pow(joystick.getLeftTriggerAxis(), 2))));
        // joystick.leftTrigger(0.05).onFalse(new InstantCommand(() -> Robot.getElevator().setSpeed(0)));
        // joystick.rightTrigger(0.05).onTrue(new InstantCommand(() -> Robot.getElevator().setSpeed(-Math.pow(joystick.getRightTriggerAxis(), 2))));
        // joystick.rightTrigger(0.05).onFalse(new InstantCommand(() -> Robot.getElevator().setSpeed(0)));
        // joystick.povUp().onTrue(new InstantCommand(() -> Robot.getArm().setSpeed(0.1))).onFalse(new InstantCommand(() -> Robot.getArm().setSpeed(0)));
        // joystick.povDown().onTrue(new InstantCommand(() -> Robot.getArm().setSpeed(-0.1))).onFalse(new InstantCommand(() -> Robot.getArm().setSpeed(0)));
        // elevator.setDefaultCommand(new RunCommand(() -> {elevator.setSpeed((joystick.getLeftTriggerAxis() > 0.05) ? Math.pow(joystick.getLeftTriggerAxis(), 2) : -Math.pow(joystick.getRightTriggerAxis(), 2));}, elevator));

        // joystick.x().whileTrue(RobotMode.alignWithLeftReef);

        joystick.start().onTrue(new InstantCommand(() -> Robot.robotMode.setCurrentMode(RobotMode.resetPose)));

        joystick.povUp().whileTrue(new SetElevatorSpeed(0.3));
        joystick.povDown().whileTrue(new SetElevatorSpeed(-0.3));

        joystick.povRight().whileTrue(new SetIntakePivotSpeed(0.3));
        joystick.povLeft().whileTrue(new SetIntakePivotSpeed(-0.3));

        joystick.a().whileTrue(new SetArmSpeed(0.3));
        joystick.y().whileTrue(new SetArmSpeed(-0.3));

        joystick.x().whileTrue(new SetClimberSpeed(0.3));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        System.out.println(autonSelector.getSelected().getName());
        return autonSelector.getSelected();
    }

}
