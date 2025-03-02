package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.RobotMode;

public class TriggerBoard {
    
    public static boolean isAutonActive() {return DriverStation.isDSAttached() ? DriverStation.isAutonomousEnabled() : false;}

    public static boolean isCoralButtonPressed() {return Robot.joystick.rightTrigger().getAsBoolean();}

    public static boolean isCoralOverrideButtonPressed() {return Robot.joystick.rightBumper().getAsBoolean();}

    public static boolean isAlgaeButtonPressed() {return Robot.joystick.leftTrigger().getAsBoolean();}

    public static boolean isAlgaeOverrideButtonPressed() {return Robot.joystick.leftBumper().getAsBoolean();}

    public static boolean isClimbControllerButtonPressed() {return Robot.joystick.leftBumper().getAsBoolean();}

    public static boolean isClimbButtonBoxButtonPressed() {
        return DashboardButtonBox.isClimbPressed();
    }

    public static boolean isResetButtonPressed() {return Robot.joystick.x().getAsBoolean();}

    public static boolean isCoralInManipulator() {
        return Robot.manipulator.hasGamePiece();
        // return Robot.manipulator.torqueCurrent() > 50;/
        // return Robot.joystick.leftTrigger().getAsBoolean();
    }

    public static boolean isAlgaeInIntake() {
        return Robot.intake.hasAlgae();
    }

    public static boolean isCollapsedTransitButtonPressed() {
        return false;
    }

    public static boolean isNearReef() {return Robot.drivetrain.isNearToReef();} 

    public static boolean isNearCoralStation() {return Robot.drivetrain.isNearToBlueCoralStation() || Robot.drivetrain.isNearToRedCoralStation();}

    public static boolean isNearCage() {return Robot.drivetrain.isNearCage();}

    public static boolean isNearProcessor() {return Robot.drivetrain.isNearProcessor();}

    public static boolean isL1Selected() {return DashboardButtonBox.getSelectedReefLevel() == 1;}

    public static boolean isL2Selected() {return DashboardButtonBox.getSelectedReefLevel() == 2;}

    public static boolean isL3Selected() {return DashboardButtonBox.getSelectedReefLevel() == 3;}

    public static boolean isL4Selected() {return DashboardButtonBox.getSelectedReefLevel() == 4;}

    public static boolean isLevelSelected() {return Robot.buttonBoxController.getRawButton(0) || Robot.buttonBoxController.getRawButton(1) || Robot.buttonBoxController.getRawButton(2) || Robot.buttonBoxController.getRawButton(3);}

    public static boolean isReefAligned() {return RobotMode.alignWithLeftReef.isAligned() || RobotMode.alignWithRightReef.isAligned();}

    public static boolean isAlgaeRemoveButtonPressed() {
        return DashboardButtonBox.isAlgaeKnockoffOn();
    }

    public static boolean hasSelectedLevelChanged() {
        return DashboardButtonBox.hasSelectedLevelChanged();
    }

    public static boolean isEndGame() {
        if (DriverStation.isDSAttached() ? DriverStation.isFMSAttached() : false) {
            return DriverStation.getMatchTime() < 20;
        } else {
            return true;
        }
    }
  
    public static boolean isCoralInIntake() {
        return Robot.intake.hasAlgae();
    }
}