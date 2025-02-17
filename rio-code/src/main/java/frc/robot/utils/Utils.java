package frc.robot.utils;

import frc.robot.Robot;

public class Utils {

    public enum ReefPosition {
        A, B, C, D, E, F, G, H, I, J, K, L
    }
    
    /**
     * converts rotations to meters using input conversion factor
     * 
     * @param rotations number of rotations as read by encoder
     * @param conversionFactor number of rotations per meter traveled
     * @return number of meters traveled
     */
    public static double rotationsToMeters(double rotations, double conversionFactor) {
        return rotations / conversionFactor;
    }

    /**
     * converts meters to rotations using input conversion factor
     * 
     * @param meters number of meters
     * @param conversionFactor number of rotations per meter traveled
     * @return number of rotations
     */
    public static double metersToRotations(double meters, double conversionFactor) {
        return meters * conversionFactor;
    }

    public static ReefPosition getSelectedReefPosition() {
        if (Robot.buttonBox.getRawButton(0)) {
            return ReefPosition.A;
        } else {
            return null;
        }
    }

}
