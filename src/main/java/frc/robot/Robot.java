// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.RobotController;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public final static Drivetrain drivetrain = TunerConstants.createDrivetrain();
  // public final static Elevator elevator = new Elevator();
  // public final static Climber climber = new Climber();
  // public final static Manipulator manipulator = new Manipulator();
  public final static Elevator elevator = null;
  public final static Climber climber = null;
  public final static Manipulator manipulator = null;
  public final static Intake intake = null;
  public final static Arm arm = null;
  public final static Limelight limelight = new Limelight("", new Pose3d(Units.inchesToMeters(12), Units.inchesToMeters(0), Units.inchesToMeters(3), new Rotation3d(0, 0, 0)));

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
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
    drivetrain.getField().setRobotPose(drivetrain.getPose());
    SmartDashboard.putData("field", drivetrain.getField());
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
}
