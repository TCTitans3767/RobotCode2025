package frc.robot.utils;

public class Utils {
    
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

}
