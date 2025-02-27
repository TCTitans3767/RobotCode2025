// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.RobotLights;
import frc.robot.subsystems.RobotMode;
import frc.robot.utils.Logger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static boolean testMode = true;

  public final static PowerDistribution pdh = new PowerDistribution();

  public final static Drivetrain drivetrain = TunerConstants.createDrivetrain();
  public final static Elevator elevator = new Elevator();
  // public final static Climber climber = new Climber();
  // public final static Manipulator manipulator = new Manipulator();
  // public final static Elevator elevator = null;
  public final static Climber climber = new Climber();
  public final static Manipulator manipulator = new Manipulator();
  public final static Intake intake = new Intake();
  public final static Arm arm = new Arm();
  public final static Limelight limelight = new Limelight("limelight-front", new Pose3d(Units.inchesToMeters(-0.548596), Units.inchesToMeters(9.66), Units.inchesToMeters(28.228805), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-23), Units.degreesToRadians(10))));
  // public final static Limelight limelight = null;
  
  public final static CommandXboxController joystick = new CommandXboxController(0);
  public final static GenericHID buttonBoxController = new GenericHID(1);
  // public final static ButtonBox buttonBox = new ButtonBox(buttonBoxController);
  public final static DashboardButtonBox buttonBox = new DashboardButtonBox();

  public final static RobotLights lights = new RobotLights();

  public final static RobotMode robotMode = new RobotMode();

  private final RobotContainer m_robotContainer;

  public Robot() {
    DogLog.setOptions(new DogLogOptions(
        () -> testMode, 
        false, 
        false, 
        false, 
        true, 
        1000
      )
    );
    m_robotContainer = new RobotContainer();
    addPeriodic(buttonBox::buttonBoxPeriodic, 0.1);
    SmartDashboard.putData(drivetrain.getField());
  }

  @Override
  public void robotInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    // buttonBox.periodic();
    // Logger.log("Selected Level", ButtonBox.getSelectedLevel());
    drivetrain.getField().setRobotPose(drivetrain.getPose());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    robotMode.setCurrentMode(RobotMode.initialTransitPose);
    robotMode.setDriveModeCommand(RobotMode.controllerDrive);
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  public static Alliance getAlliance() {
    return DriverStation.isDSAttached() ? DriverStation.getAlliance().orElse(Alliance.Blue) : Alliance.Blue;
  }

  public static Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public static Limelight getLimelight() {
    return limelight;
  }

  public static Elevator getElevator() {
    return elevator;
  }

  public static Manipulator getManipulator() {
      return manipulator;
  }

  public static Climber getClimber() {
      return climber;
  }

  public static Intake getIntake() {
    return intake;
  }

  public static Arm getArm() {
      return arm;
  }

  public static void enableTestMode() {
      Robot.testMode = true;
  }

  public static void disableTestMode() {
      Robot.testMode = false;
  }
}
