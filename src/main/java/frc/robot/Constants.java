package frc.robot;

public class Constants {
    
    public final class ReefTagIDs {

        public final static Integer[] blue = {
            17,
            18,
            19,
            20,
            21,
            22
        };

        public final static Integer[] red = {
            6,
            7,
            8,
            9,
            10,
            11
        };

    }

    public final class Climber {
        public final static int leftMotorID = 0;
        public final static int rightMotorID = 1;

        public final static double kP = 0.1;
        public final static double kI = 0.1;
        public final static double kD = 0.1;
        public final static double kG = 0.1;
        public final static double kV = 0.1;
        public final static double kS = 0.1;

        public final static double maxVelocity = 0.1;
        public final static double maxAcceleration = 0.1;

        public final static double conversonFactor = 1;
    }

    public final class Robot {
        public final static double chassisWidth = 26.0;
        public final static double chassisDepth = 26.0;
    }

}
