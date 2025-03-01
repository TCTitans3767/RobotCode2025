package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;

public class Constants {
    public static final class NetworkTables {
        public static final String buttonBoxTable = "Dashboard ButtonBox";
        public static final String coralStationAlignRightTopic = "CoralStationAlignRight";
        public static final String ejectAlgaeTopic = "EjectAlgae";
        public static final String selectedReefBranchTopic = "SelectedReefBranch";
        public static final String selectedReefLevelTopic = "SelectedReefLevel";
        public static final boolean initialCoralStationAlignRight = false;
        public static final boolean initialEjectAlgae = true;
        public static final ReefBranch initialReefBranch = ReefBranch.A;
        public static final ReefLevel initialReefLevel = ReefLevel.L4;
    }

    public final class ButtonBoxButtons {
        public final static double debounceSeconds = 0.2;

        public final static Map<ReefBranch, Integer> branchButtonMap;
        public final static Map<ReefLevel, Integer> levelButtonMap;

        static {
            branchButtonMap = new HashMap<ReefBranch, Integer>();
            branchButtonMap.put(ReefBranch.A, 1);
            branchButtonMap.put(ReefBranch.B, 2);
            branchButtonMap.put(ReefBranch.C, 3);
            branchButtonMap.put(ReefBranch.D, 4);
            branchButtonMap.put(ReefBranch.E, 5);
            branchButtonMap.put(ReefBranch.F, 6);
            branchButtonMap.put(ReefBranch.G, 7);
            branchButtonMap.put(ReefBranch.H, 8);
            branchButtonMap.put(ReefBranch.I, 9);
            branchButtonMap.put(ReefBranch.J, 10);
            branchButtonMap.put(ReefBranch.K, 11);
            branchButtonMap.put(ReefBranch.L, 12);

            levelButtonMap = new HashMap<ReefLevel, Integer>();
            levelButtonMap.put(ReefLevel.L1, 13);
            levelButtonMap.put(ReefLevel.L2, 14);
            levelButtonMap.put(ReefLevel.L3, 15);
            levelButtonMap.put(ReefLevel.L4, 16);
        }

        public final static int algaeEject = 17;
        public final static int coralStationAlignRight = 18;
        public final static int climb = 19;
    }

    public static final class ReefTagIDs {
        public final static int blueReefAB = 18;
        public final static int blueReefCD = 17;
        public final static int blueReefEF = 22;
        public final static int blueReefGH = 21;
        public final static int blueReefIJ = 20;
        public final static int blueReefKL = 19;

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

    public static final class Field {
        public final static Translation2d blueReefCenter = new Translation2d(4.5, 4.03);
        public final static Translation2d redReefCenter = new Translation2d(13.08, 4.03);
        public final static Translation2d blueRightCoralStation = new Translation2d(0.9, 0.6);
        public final static Pose2d blueRightCoralStationRightAlignment = new Pose2d(new Translation2d(0.9, 0.6), new Rotation2d(Units.degreesToRadians(126.5)));
        public final static Pose2d blueRightCoralStationLeftAlignment = new Pose2d(new Translation2d(0.9, 0.6), new Rotation2d(Units.degreesToRadians(126.5)));
        public final static Translation2d redRightCoralStation = new Translation2d(16.68, 7.4);
        public final static Pose2d redRightCoralStationRightAlignment = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        public final static Pose2d redRightCoralStationLeftAlignment  = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        public final static Translation2d blueLeftCoralStation = new Translation2d(0.9, 7.41);
        public final static Pose2d blueLeftCoralStationRightAlignment = new Pose2d(new Translation2d(0.9, 7.41), new Rotation2d(Units.degreesToRadians(233.5)));
        public final static Pose2d blueLeftCoralStationLeftAlignment  = new Pose2d(new Translation2d(0.9, 7.41), new Rotation2d(Units.degreesToRadians(233.5)));
        public final static Translation2d redLeftCoralStation = new Translation2d(16.68, 0.58);
        public final static Pose2d redLeftCoralStationRightAlignment = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        public final static Pose2d redLeftCoralStationLeftAlignment  = new Pose2d(new Translation2d(0, 0), new Rotation2d());
    }
  
    public static final class Climber {
        public final static int leftMotorID = 18;
        public final static int rightMotorID = 19;
      
        public final static double maxVelocity = 0.1;
        public final static double maxAcceleration = 0.1;

        public final static double conversonFactor = 1;
    }
  
    public static final class Elevator {
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
        public final static double metersMin = 0.020;

        public final static double errorTolerance = 0.01;
        public final static double zeroingSpeed = -0.05;
        public final static double zeroingThreshold = 0.01;
    }

    public static final class Manipulator {
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

        public static final double detectionRange = 0.066;
        public static final double sensorDebounce = 0.004;
        public static final int sensorID = 24;
    }


    public static final class Arm {
        // CAN IDs
        public final static int ArmMotorID = 15;
        public static final int ArmEncoderID = 16;

        // PID Constants
        public final static double kP = 68;
        public final static double kI = 0;
        public final static double kD = 0;
        public final static double kG = 0.4;
        public final static double kV = 0;
        public final static double kS = 0.9;
      
        // Motion Magic Constants
        public final static double maxVelocity = 6;
        public final static double maxAcceleration = 4.5;
        public final static double rotationsMax = 0.6;
        public final static double rotationsMin = -0.6;
      
        public final static double conversionFactor = 1;

        public final static double errorTolerance = 0.1;
    }

    public static final class Intake {
        // CAN IDs
        public final static int leftWheelMotorID = 21;
        public static final int rightWheelMotorID = 22;
        public final static int pivotMotorID = 20;
        public final static int pivotEncoderID = 23;

        // PID Constants
        public final static double kP = 70;
        public final static double kI = 0;
        public final static double kD = 0.1;
        public final static double kG = 0.15;
        public final static double kV = 0;
        public final static double kS = 0.32;


        // Motion Magic Constants
        public final static double maxVelocity = 5;
        public final static double maxAcceleration = 4;
        public final static double angleMax = 0.1;
        public final static double angleMin = 0.1;
      
        // Conversion Factor
        public final static double pivotConversionFactor = 1;
        public final static double wheelConversionFactor = 1;
        public static final double pivotErrorTolerance = 0.01;

        public static final double detectionRange = 0.1;
        public static final double sensorDebounce = 0.01;
        public static final int sensorID = 25;
        public static final double wheelCurrentLimit = 60;
        public static final double pivotCurrentLimit = 80;

    }
  

    public static final class Robot {
        public final static double chassisWidthMeters = Units.inchesToMeters(32.5);
        public final static double chassisDepthMeters = Units.inchesToMeters(32.5);
    }

    public static final class Drive {
        public final static double reefDistanceThreshold = 3;
        public final static double coralStationDistanceThreshold = 1.5;


        public final static double YAlignmentTolerance = 0.02;
        public final static double YAlignementPIDkP = 0.9;
        public final static double YAlignementPIDkI = 0;
        public final static double YAlignementPIDkD = 0;
        public final static double YFeedForward = 0;

        public final static double XAlignmentTolerance = 0.02;
        public final static double XAlignementPIDkP = 0.5;
        public final static double XAlignementPIDkI = 0;
        public final static double XAlignementPIDkD = 0;
        public final static double XFeedForward = 0;

        public final static double rotationAlignementPIDkP = 0.0135;
        public final static double rotationAlignementPIDkI = 0;
        public final static double rotationAlignementPIDkD = 0;
        public static final double rotationalFeedForward = 0.0015;
        public final static double headingAlignmentTolerance = 0.01;

        public static final double L4XAlignementPIDkP = 0.5;
        public static final double L4XAlignementPIDkI = 0;
        public static final double L4XAlignementPIDkD = 0;

        public static final double L4YAlignementPIDkP = 0.85;
        public static final double L4YAlignementPIDkI = 0;
        public static final double L4YAlignementPIDkD = 0;

        public static final double L4RotationalAlignementPIDkP = 0.0135;
        public static final double L4RotationalAlignementPIDkI = 0;
        public static final double L4RotationalAlignementPIDkD = 0.0015;

    }

    public final static class ButtonBoxBindings {
        public final static int A = 1;
        public final static int B = 2;
        public final static int C = 3;
        public final static int D = 4;
        public final static int E = 5;
        public final static int F = 6;
        public final static int G = 7;
        public final static int H = 8;
        public final static int I = 9;
        public final static int J = 10;
        public final static int K = 11;
        public final static int L = 12;

        public final static int L1 = 13;
        public final static int L2 = 14;
        public final static int L3 = 15;
        public final static int L4 = 16;
        public static final int climb = 0;
    }

}
