package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Logger;

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


public class Manipulator extends SubsystemBase{
    private final TalonFX motor;
    private final TalonFXConfiguration config;

    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;

    private final CANrange manipulatorSensor;
    private final CANrangeConfiguration manipulatorSensorConfig;

    public Manipulator() {
        // Motor setup
        motor = new TalonFX(frc.robot.Constants.Manipulator.motorID);
        config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = Constants.Manipulator.conversionFactor;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Slot 0 PID setup
        slot0Config = new Slot0Configs();
        slot0Config.kP = Constants.Manipulator.kP;
        slot0Config.kI = Constants.Manipulator.kI;
        slot0Config.kD = Constants.Manipulator.kD;
        slot0Config.kG = Constants.Manipulator.kG;
        slot0Config.kV = Constants.Manipulator.kV;
        slot0Config.kS = Constants.Manipulator.kS;
        slot0Config.GravityType = GravityTypeValue.Arm_Cosine;
        
        // Motion Magic setup
        motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.Manipulator.maxVelocity;
        motionMagicConfig.MotionMagicAcceleration = Constants.Manipulator.maxAcceleration;

        // Set the configurations
        motor.getConfigurator().apply(config);
        motor.getConfigurator().apply(slot0Config);
        motor.getConfigurator().apply(motionMagicConfig);
        motor.setNeutralMode(NeutralModeValue.Brake);

        manipulatorSensor = new CANrange(Constants.Manipulator.sensorID);
        manipulatorSensorConfig = new CANrangeConfiguration();
        manipulatorSensorConfig.ProximityParams.ProximityThreshold = Constants.Manipulator.detectionRange;
        manipulatorSensorConfig.ProximityParams.ProximityHysteresis = Constants.Manipulator.sensorDebounce;
        manipulatorSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        manipulatorSensor.getConfigurator().apply(manipulatorSensorConfig);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.log("Manipulator/Has Game Piece", hasGamePiece());
        Logger.log("Manipulator/Sensor Measured Distance", manipulatorSensor.getDistance().getValueAsDouble());
    }

    public double torqueCurrent() {
        return motor.getTorqueCurrent().getValueAsDouble();
    }

    public void setSpeed(double speed) {
        // Move the manipulator at the given speed
        motor.set(speed);
    }

    public boolean hasGamePiece() {
        return manipulatorSensor.getIsDetected().getValue();
    }
}
