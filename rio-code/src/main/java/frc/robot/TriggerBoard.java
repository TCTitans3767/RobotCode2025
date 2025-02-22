package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.RobotMode;

public class TriggerBoard {
    
    public static boolean isCoralButtonPressed() {return Robot.joystick.rightTrigger().getAsBoolean();}

    public static boolean isAlgaeButtonPressed() {return Robot.joystick.rightBumper().getAsBoolean();}

    public static boolean isClimbButtonPressed() {return false;}

    public static boolean isResetButtonPressed() {return Robot.joystick.x().getAsBoolean();}

    public static boolean isCoralInManipulator() {
        return Robot.manipulator.hasGamePiece();
        // return Robot.manipulator.torqueCurrent() > 50;/
        // return Robot.joystick.leftTrigger().getAsBoolean();
    }

    public static boolean isAlgaeInIntake() {
        // return Robot.intake.hasGamePiece();
        return false;
    }

    public static boolean isCollapsedTransitButtonPressed() {
        return false;
    }

    public static boolean isNearReef() {return Robot.drivetrain.isNearToReef();} 

    public static boolean isNearCoralStation() {return Robot.drivetrain.isNearToCoralStation();}

    public static boolean isNearCage() {return Robot.drivetrain.isNearCage();}

    public static boolean isNearProcessor() {return Robot.drivetrain.isNearProcessor();}

    public static boolean isL1Selected() { return ButtonBox.getSelectedLevel() == ReefLevel.L1; }
    public static boolean isL2Selected() { return ButtonBox.getSelectedLevel() == ReefLevel.L2; }
    public static boolean isL3Selected() { return ButtonBox.getSelectedLevel() == ReefLevel.L3; }
    public static boolean isL4Selected() { return ButtonBox.getSelectedLevel() == ReefLevel.L4; }

    public static boolean isReefAligned() {return RobotMode.alignWithLeftReef.isAligned() || RobotMode.alignWithRightReef.isAligned();}
}
