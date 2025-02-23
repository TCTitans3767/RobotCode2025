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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static boolean testMode = true;

  public final static PowerDistribution pdh = new PowerDistribution();

  public final static Drivetrain drivetrain = TunerConstants.createDrivetrain();

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
    SmartDashboard.putData(drivetrain.getField());
  }

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

  public static void enableTestMode() {
      Robot.testMode = true;
  }

  public static void disableTestMode() {
      Robot.testMode = false;
  }
}
