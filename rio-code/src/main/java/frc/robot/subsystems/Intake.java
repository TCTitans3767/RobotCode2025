package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;


public class Intake extends SubsystemBase{
    private final TalonFX leftWheelMotor, rightWheelMotor, pivotMotor;
    private final TalonFXConfiguration leftWheelConfig, rightWheelConfig, pivotConfig;

    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;
    
    private final CANrange intakeSensor;
    private final CANrangeConfiguration intakeSensorConfig;

    private double targetRotations = 0;

    public Intake() {
        // Motor basic setup
        leftWheelMotor = new TalonFX(Constants.Intake.leftWheelMotorID);
        leftWheelConfig = new TalonFXConfiguration();
        leftWheelConfig.Feedback.SensorToMechanismRatio = Constants.Intake.wheelConversionFactor;
        leftWheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rightWheelMotor = new TalonFX(Constants.Intake.rightWheelMotorID);
        rightWheelConfig = new TalonFXConfiguration();
        rightWheelConfig.Feedback.SensorToMechanismRatio = Constants.Intake.wheelConversionFactor;
        rightWheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        pivotMotor = new TalonFX(Constants.Intake.pivotMotorID);
        pivotConfig = new TalonFXConfiguration();
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.Intake.pivotConversionFactor;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
         
        // Slot 0 PID setup 
        slot0Config = new Slot0Configs();
        slot0Config.kP = Constants.Intake.kP;
        slot0Config.kI = Constants.Intake.kI;
        slot0Config.kD = Constants.Intake.kD;
        slot0Config.kG = Constants.Intake.kG;
        slot0Config.kV = Constants.Intake.kV;
        slot0Config.kS = Constants.Intake.kS;
        slot0Config.GravityType = GravityTypeValue.Arm_Cosine;

        // Motion Magic setup
        motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.Intake.maxVelocity;
        motionMagicConfig.MotionMagicAcceleration = Constants.Intake.maxAcceleration;

        // Set the configurations
        leftWheelMotor.getConfigurator().apply(leftWheelConfig);
        leftWheelMotor.setNeutralMode(NeutralModeValue.Brake);

        rightWheelMotor.getConfigurator().apply(rightWheelConfig);
        rightWheelMotor.setNeutralMode(NeutralModeValue.Brake);
        rightWheelMotor.setControl(new Follower(Constants.Intake.leftWheelMotorID, true));

        intakeSensor = new CANrange(Constants.Intake.sensorID);
        intakeSensorConfig = new CANrangeConfiguration();
        intakeSensorConfig.ProximityParams.ProximityThreshold = Constants.Intake.detectionRange;
        intakeSensorConfig.ProximityParams.ProximityHysteresis = Constants.Intake.sensorDebounce;
        intakeSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        intakeSensor.getConfigurator().apply(intakeSensorConfig);
        
        pivotMotor.getConfigurator().apply(slot0Config);
        pivotMotor.getConfigurator().apply(motionMagicConfig);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        Logger.log("Intake/Pivot Position", pivotMotor.getPosition().getValueAsDouble());
        Logger.log("Intake/Is Pivot At Position", isPivotAtPosition());
        Logger.log("Intake/Has Game Piece", hasGamePiece());
    }
    
    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void setWheelSpeed(double speed) {
        leftWheelMotor.set(speed);
    }

    public void setPivotPosition(double rotations) {
        targetRotations = rotations;
        pivotMotor.setControl(new MotionMagicVoltage(rotations));
    }

    public boolean isPivotAtPosition() {
        return MathUtil.isNear(targetRotations, pivotMotor.getPosition().getValueAsDouble(), Constants.Intake.pivotErrorTolerance);
    }

    public boolean hasGamePiece() {
        return intakeSensor.getIsDetected().getValue();
    }
}
