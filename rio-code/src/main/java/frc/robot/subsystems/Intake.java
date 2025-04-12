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
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;


public class Intake extends SubsystemBase{
    private final TalonFX leftWheelMotor, rightWheelMotor, pivotMotor;
    private final TalonFXConfiguration leftWheelConfig, rightWheelConfig, pivotConfig;

    private final Slot0Configs PivotSlot0Config, leftWheelSlot0Config, rightWheelSlot0Config;
    private final MotionMagicConfigs motionMagicConfig, wheelMotionMagicConfig;
    
    private final CANrange intakeSensor;
    private final CANrangeConfiguration intakeSensorConfig;

    private final CANcoder pivotEncoder;

    private double targetRotations = 0;

    public Intake() {
        // Motor basic setup
        leftWheelMotor = new TalonFX(Constants.Intake.leftWheelMotorID);
        leftWheelConfig = new TalonFXConfiguration();
        leftWheelConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.wheelCurrentLimit;
        leftWheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftWheelConfig.Feedback.SensorToMechanismRatio = Constants.Intake.wheelConversionFactor;
        leftWheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rightWheelMotor = new TalonFX(Constants.Intake.rightWheelMotorID);
        rightWheelConfig = new TalonFXConfiguration();
        rightWheelConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.wheelCurrentLimit;
        rightWheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightWheelConfig.Feedback.SensorToMechanismRatio = Constants.Intake.wheelConversionFactor;
        rightWheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        pivotEncoder = new CANcoder(Constants.Intake.pivotEncoderID);

        pivotMotor = new TalonFX(Constants.Intake.pivotMotorID);
        pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.pivotCurrentLimit;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.Feedback.FeedbackRemoteSensorID = Constants.Intake.pivotEncoderID;
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfig.Feedback.RotorToSensorRatio = Constants.Intake.pivotConversionFactor;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
         
        // Slot 0 PID setup 
        PivotSlot0Config = new Slot0Configs();
        PivotSlot0Config.kP = Constants.Intake.kP;
        PivotSlot0Config.kI = Constants.Intake.kI;
        PivotSlot0Config.kD = Constants.Intake.kD;
        PivotSlot0Config.kG = Constants.Intake.kG;
        PivotSlot0Config.kV = Constants.Intake.kV;
        PivotSlot0Config.kS = Constants.Intake.kS;
        PivotSlot0Config.GravityType = GravityTypeValue.Arm_Cosine;

        // Slot 0 PID setup 
        leftWheelSlot0Config = new Slot0Configs();
        leftWheelSlot0Config.kP = Constants.Intake.wheelkP;
        leftWheelSlot0Config.kI = Constants.Intake.wheelkI;
        leftWheelSlot0Config.kD = Constants.Intake.wheelkD;
        leftWheelSlot0Config.kG = Constants.Intake.wheelkG;
        leftWheelSlot0Config.kV = Constants.Intake.wheelkV;
        leftWheelSlot0Config.kS = Constants.Intake.wheelkS;
        leftWheelSlot0Config.GravityType = GravityTypeValue.Elevator_Static;

        // Slot 0 PID setup 
        rightWheelSlot0Config = new Slot0Configs();
        rightWheelSlot0Config.kP = Constants.Intake.wheelkP;
        rightWheelSlot0Config.kI = Constants.Intake.wheelkI;
        rightWheelSlot0Config.kD = Constants.Intake.wheelkD;
        rightWheelSlot0Config.kG = Constants.Intake.wheelkG;
        rightWheelSlot0Config.kV = Constants.Intake.wheelkV;
        rightWheelSlot0Config.kS = Constants.Intake.wheelkS;
        rightWheelSlot0Config.GravityType = GravityTypeValue.Elevator_Static;

        // Motion Magic setup
        motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.Intake.maxVelocity;
        motionMagicConfig.MotionMagicAcceleration = Constants.Intake.maxAcceleration;

        // Motion Magic setup
        wheelMotionMagicConfig = new MotionMagicConfigs();
        wheelMotionMagicConfig.MotionMagicCruiseVelocity = Constants.Intake.wheelMaxVelocity;
        wheelMotionMagicConfig.MotionMagicAcceleration = Constants.Intake.wheelMaxAcceleration;

        // Set the configurations
        leftWheelMotor.getConfigurator().apply(leftWheelConfig);
        leftWheelMotor.getConfigurator().apply(leftWheelSlot0Config);
        leftWheelMotor.getConfigurator().apply(wheelMotionMagicConfig);
        leftWheelMotor.setNeutralMode(NeutralModeValue.Brake);

        rightWheelMotor.getConfigurator().apply(rightWheelConfig);
        rightWheelMotor.getConfigurator().apply(rightWheelSlot0Config);
        rightWheelMotor.getConfigurator().apply(wheelMotionMagicConfig);
        rightWheelMotor.setNeutralMode(NeutralModeValue.Brake);
        rightWheelMotor.setControl(new Follower(Constants.Intake.leftWheelMotorID, true));

        intakeSensor = new CANrange(Constants.Intake.sensorID);
        intakeSensorConfig = new CANrangeConfiguration();
        intakeSensorConfig.ProximityParams.ProximityThreshold = Constants.Intake.detectionRange;
        intakeSensorConfig.ProximityParams.ProximityHysteresis = Constants.Intake.sensorDebounce;
        intakeSensorConfig.FovParams.FOVRangeX = Constants.Intake.fovXRange;
        intakeSensorConfig.FovParams.FOVRangeY = Constants.Intake.fovYRange;
        intakeSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        intakeSensor.getConfigurator().apply(intakeSensorConfig);

        pivotMotor.getConfigurator().apply(pivotConfig);        
        pivotMotor.getConfigurator().apply(PivotSlot0Config);
        pivotMotor.getConfigurator().apply(motionMagicConfig);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        Logger.log("Intake/Pivot Position", pivotMotor.getPosition().getValueAsDouble());
        Logger.log("Intake/Pivot Absolute Position", pivotEncoder.getPosition().getValueAsDouble());
        Logger.log("Intake/Is Pivot At Position", isPivotAtPosition());
        Logger.log("Intake/Stator Current", leftWheelMotor.getStatorCurrent().getValueAsDouble());
        Logger.log("Intake/Wheel Velocity", leftWheelMotor.getVelocity().getValueAsDouble());
        Logger.log("Intake/Has Game Piece", hasCoral());
    }

    public TalonFX getLeftWheelMotor() {
        return leftWheelMotor;
    }
    
    public TalonFX getRightWheelMotor() {
        return rightWheelMotor;
    }

    public void scoreL1() {
        leftWheelMotor.setControl(new MotionMagicVelocityVoltage(-30));
        rightWheelMotor.setControl(new MotionMagicVelocityVoltage(-16));
    }

    public double getPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }
    
    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void resetWheelSpeed() {
        leftWheelMotor.set(0);
        rightWheelMotor.set(0);
    }

    public void setWheelSpeed(double rotationsPerSecond) {
        leftWheelMotor.setControl(new MotionMagicVelocityVoltage(rotationsPerSecond));
        rightWheelMotor.setControl(new MotionMagicVelocityVoltage(rotationsPerSecond));
    }

    public void setWheelPower(double speed) {
        leftWheelMotor.set(speed);
        rightWheelMotor.set(speed);
    }

    public void setPivotPosition(double rotations) {
        targetRotations = rotations;
        pivotMotor.setControl(new MotionMagicVoltage(rotations));
        Logger.log("Intake/Pivot Target", rotations);
    }

    public boolean isPivotAtPosition() {
        return MathUtil.isNear(targetRotations, pivotMotor.getPosition().getValueAsDouble(), Constants.Intake.pivotErrorTolerance);
    }

    public boolean hasAlgae() {
        return leftWheelMotor.getStatorCurrent().getValueAsDouble() >= 70;
    }

    public boolean hasCoral() {
        return intakeSensor.getIsDetected().getValue();
    }

    public boolean isWheelMotorTooHot() {
        return leftWheelMotor.getDeviceTemp().getValueAsDouble() >= 70;
    }

    public double getWheelSpeed() {
        return leftWheelMotor.get();
    }

    public void setPivotPosition(double rotations, double maximumVelocity) {
        targetRotations = rotations;
        pivotMotor.setControl(new MotionMagicVoltage(rotations));
        Logger.log("Intake/Pivot Target", rotations);
    }
}
