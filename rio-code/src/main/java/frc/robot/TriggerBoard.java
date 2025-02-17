package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TriggerBoard {
    
    public static boolean isCoralButtonPressed() {return Robot.joystick.rightTrigger().getAsBoolean();}

    public static boolean isAlgaeButtonPressed() {return false;}

    public static boolean isClimbButtonPressed() {return false;}

    public static boolean isResetButtonPressed() {return false;}

    public static boolean isCoralInManipulator() {return Robot.joystick.leftTrigger().getAsBoolean();}

    public static boolean isAlgaeInIntake() {return false;}

    public static boolean isNearReef() {return false;}

    public static boolean isNearCoralStation() {return false;}

    public static boolean isNearCage() {return false;}

    public static boolean isNearProcessor() {return false;}

    public static boolean isL1Selected() {return false;}

    public static boolean isL2Selected() {return false;}

    public static boolean isL3Selected() {return false;}

    public static boolean isL4Selected() {return false;}

    public static boolean isLevelSelected() {return false;}

    public static boolean isReefAligned() {return Robot.robotMode.currentDriveMode.getName() == "AlignWithLeftReef" || Robot.robotMode.currentDriveMode.getName() == "AlignWithRightReef";}
}
