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
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
<<<<<<< Updated upstream
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======
import edu.wpi.first.wpilibj.LEDPattern;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.RobotMode;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static boolean testMode = true;

  public final static PowerDistribution pdh = new PowerDistribution();

  public final static Drivetrain drivetrain = TunerConstants.createDrivetrain();
  public final static Elevator elevator = new Elevator();
  // public final static Climber climber = new Climber();
  // public final static Manipulator manipulator = new Manipulator();
  // public final static Elevator elevator = null;
  public final static Climber climber = null;
  public final static Manipulator manipulator = null;
  public final static Intake intake = null;
  public final static Arm arm = null;
  public final static Limelight limelight = new Limelight("", new Pose3d(Units.inchesToMeters(12), Units.inchesToMeters(0), Units.inchesToMeters(3), new Rotation3d(0, 0, 0)));
  // public final static Limelight limelight = null;
  
  public final static CommandXboxController joystick = new CommandXboxController(0);

  public final static RobotMode robotMode = new RobotMode();

  private final RobotContainer m_robotContainer;
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
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
<<<<<<< Updated upstream
    SmartDashboard.putData(drivetrain.getField());
  }
=======
      /** Called once at the beginning of the robot program. */
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
      m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
      m_ledBuffer = new AddressableLEDBuffer(60);
      m_led.setLength(m_ledBuffer.getLength());

    // Set the data
      m_led.setData(m_ledBuffer);
      m_led.start();

      // Create an LED pattern that sets the entire strip to solid red
      LEDPattern red = LEDPattern.solid(Color.kRed);

// Apply the LED pattern to the data buffer
      red.applyTo(m_ledBuffer);

// Write the data to the LED strip
      m_led.setData(m_ledBuffer);
 }
>>>>>>> Stashed changes

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
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
