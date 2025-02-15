package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.util.Units;

public class Constants {
    
    public final class ReefTagIDs {

        public final static int blueReefAB = 18;
        public final static int blueReefCD = 19;
        public final static int blueReefEF = 22;
        public final static int blueReefGH = 21;
        public final static int blueReefIJ = 20;
        public final static int blueReefKL = 17;

        public final static int redReefAB = 7;
        public final static int redReefCD = 8;
        public final static int redReefEF = 9;
        public final static int redReefGH = 10;
        public final static int redReefIJ = 11;
        public final static int redReefKL = 6;

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
      
        public final static double maxVelocity = 0.1;
        public final static double maxAcceleration = 0.1;

        public final static double conversonFactor = 1;
    }
  
    public final class Elevator {
        // CAN IDs
        public final static int leftMotorID = 14;
        public final static int rightMotorID = 15;

        // PID Constants
        public final static double kP = 0.1;
        public final static double kI = 0.1;
        public final static double kD = 0.1;
        public final static double kG = 0.1;
        public final static double kV = 0.1;
        public final static double kS = 0.1;
        
        // Motion Magic Constants
        public final static double maxVelocity = 0.1;
        public final static double maxAcceleration = 0.1;

        // Conversion Factor
        public final static double conversionFactor = 1;

        public final static double RotationsPerMeter = 39.997;
    }

    public final class Manipulator {
        // CAN IDs
        public final static int motorID = 2;

        // PID Constants
        public final static double kP = 0.1;
        public final static double kI = 0.1;
        public final static double kD = 0.1;
        public final static double kG = 0.1;
        public final static double kV = 0.1;
        public final static double kS = 0.1;

        // Motion Magic Constants
        public final static double maxVelocity = 0.1;
        public final static double maxAcceleration = 0.1;

        // Conversion Factor
        public final static double conversionFactor = 1;
    }


    public final class Arm {
        // CAN IDs
        public final static int ArmMotorID = 16;
        // PID Constants
        public final static double kP = 0.1;
        public final static double kI = 0.1;
        public final static double kD = 0.1;
        public final static double kG = 0.1;
        public final static double kV = 0.1;
        public final static double kS = 0.1;
      
        // Motion Magic Constants
        public final static double maxVelocity = 0.1;
        public final static double maxAcceleration = 0.1;
        public final static double angleMax = 0.1;
        public final static double angleMin = 0.1;
      
        public final static double conversionFactor = 1;
        public static final int ArmEncoderID = 0;
    }

    public final class Intake {
        // CAN IDs
        public final static int wheelMotorID = 2;
        public final static int pivotMotorID = 3;

        // PID Constants
        public final static double kP = 0.1;
        public final static double kI = 0.1;
        public final static double kD = 0.1;
        public final static double kG = 0.1;
        public final static double kV = 0.1;
        public final static double kS = 0.1;


         // Motion Magic Constants
        public final static double maxVelocity = 0.1;
        public final static double maxAcceleration = 0.1;
        public final static double angleMax = 0.1;
        public final static double angleMin = 0.1;
      
        // Conversion Factor
        public final static double conversionFactor = 1;

    }
  
    public final class Robot {
        public final static double chassisWidthMeters = Units.inchesToMeters(26.0);
        public final static double chassisDepthMeters = Units.inchesToMeters(26.0);
    }

}
