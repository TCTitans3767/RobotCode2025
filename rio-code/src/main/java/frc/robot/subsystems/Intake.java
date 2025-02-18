package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
    private final TalonFX wheelMotor, pivotMotor;
    private final TalonFXConfiguration wheelConfig, pivotConfig;

    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;
    
    private final CANrange intakeSensor;
    private final CANrangeConfiguration intakeSensorConfig;

    private double targetRotations = 0;

    public Intake() {
        // Motor basic setup
        wheelMotor = new TalonFX(Constants.Intake.wheelMotorID);
        wheelConfig = new TalonFXConfiguration();
        wheelConfig.Feedback.SensorToMechanismRatio = Constants.Intake.conversionFactor;
        wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotMotor = new TalonFX(Constants.Intake.pivotMotorID);
        pivotConfig = new TalonFXConfiguration();
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.Intake.conversionFactor;
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
        wheelMotor.getConfigurator().apply(wheelConfig);
        wheelMotor.setNeutralMode(NeutralModeValue.Brake);

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
        // This method will be called once per scheduler run
    }
    
    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void setWheelSpeed(double speed) {
        wheelMotor.set(speed);
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
