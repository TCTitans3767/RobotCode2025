package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    public final class Field {
        public final static Translation2d reefCenter = new Translation2d(0, 0);
        public final static Translation2d blueRightCoralStation = new Translation2d(0, 0);
        public final static Translation2d redRightCoralStation = new Translation2d(0, 0);
        public final static Translation2d blueLeftCoralStation = new Translation2d(0, 0);
        public final static Translation2d redLeftCoralStation = new Translation2d(0, 0);
    }
  
    public final class Climber {
        public final static int leftMotorID = 18;
        public final static int rightMotorID = 19;
      
        public final static double maxVelocity = 0.1;
        public final static double maxAcceleration = 0.1;

        public final static double conversonFactor = 1;
    }
  
    public final class Elevator {
        // CAN IDs
        public final static int leftMotorID = 13;
        public final static int rightMotorID = 14;

        // PID Constants
        public final static double kP = 13;
        public final static double kI = 0;
        public final static double kD = 0;
        public final static double kG = 0.185;
        public final static double kV = 0;
        public final static double kS = 0.5;
        
        // Motion Magic Constants
        public final static double maxVelocity = 300;
        public final static double maxAcceleration = 200;

        // Conversion Factor
        public final static double conversionFactor = 1;

        public final static double RotationsPerMeter = 39.997;

        public final static double metersMax = 1.125;
        public final static double metersMin = 0.050;

        public final static double errorTolerance = 0.01;
        public final static double zeroingSpeed = -0.05;
        public final static double zeroingThreshold = 0.01;
    }

    public final class Manipulator {
        // CAN IDs
        public final static int motorID = 17;

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

        public static final double detectionRange = 0.02;
        public static final double sensorDebounce = 0.001;
        public static final int sensorID = 18;
    }


    public final class Arm {
        // CAN IDs
        public final static int ArmMotorID = 15;
        public static final int ArmEncoderID = 16;

        // PID Constants
        public final static double kP = 58;
        public final static double kI = 0;
        public final static double kD = 0;
        public final static double kG = 0.4;
        public final static double kV = 0;
        public final static double kS = 0.9;
      
        // Motion Magic Constants
        public final static double maxVelocity = 4;
        public final static double maxAcceleration = 3.5;
        public final static double rotationsMax = 0.6;
        public final static double rotationsMin = -0.6;
      
        public final static double conversionFactor = 1;

        public final static double errorTolerance = 0.1;
    }

    public final class Intake {
        // CAN IDs
        public final static int leftWheelMotorID = 21;
        public static final int rightWheelMotorID = 22;
        public final static int pivotMotorID = 20;

        // PID Constants
        public final static double kP = 0;
        public final static double kI = 0;
        public final static double kD = 0;
        public final static double kG = 0;
        public final static double kV = 0;
        public final static double kS = 0;


        // Motion Magic Constants
        public final static double maxVelocity = 4;
        public final static double maxAcceleration = 3.5;
        public final static double angleMax = 0.1;
        public final static double angleMin = 0.1;
      
        // Conversion Factor
        public final static double pivotConversionFactor = 1;
        public final static double wheelConversionFactor = 1;
        public static final double pivotErrorTolerance = 0.1;

        public static final double detectionRange = 0.1;
        public static final double sensorDebounce = 0.01;
        public static final int sensorID = 19;

    }
  
    public final class Robot {
        public final static double chassisWidthMeters = Units.inchesToMeters(26.0);
        public final static double chassisDepthMeters = Units.inchesToMeters(26.0);
    }

    public final class Drive {
        public final static double alignmentTolerance = 0.02;
    }

}
