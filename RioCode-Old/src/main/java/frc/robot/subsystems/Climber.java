package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Climber extends SubsystemBase {
    private TalonFX rightMotor, leftMotor;
    private final TalonFXConfiguration rightConfig, leftConfig;
    // Private final TalonFXConfiguration rightConfig, leftConfig;
    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;

    public Climber() {
        // Initialize the motors
        leftMotor = new TalonFX(Constants.Climber.leftMotorID); 
        leftConfig = new TalonFXConfiguration();
        leftConfig.Feedback.SensorToMechanismRatio = Constants.Climber.conversonFactor;
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        slot0Config = new Slot0Configs();
        slot0Config.kP = 0.1;
        slot0Config.kI = 0.1;
        slot0Config.kD = 0.1;
        slot0Config.GravityType = GravityTypeValue.Arm_Cosine;
        slot0Config.kG = 0.1; // Gravity 
        slot0Config.kS = 0.1;


        motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.Climber.maxVelocity;
        motionMagicConfig.MotionMagicAcceleration = Constants.Climber.maxAcceleration;


        leftMotor.getConfigurator().apply(leftConfig);
        leftMotor.getConfigurator().apply(slot0Config);
        leftMotor.getConfigurator().apply(motionMagicConfig);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);

        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor = new TalonFX(Constants.Climber.rightMotorID);
        rightMotor.getConfigurator().apply(rightConfig);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setControl(new Follower(Constants.Climber.leftMotorID, true));
    }

    @Override
    public void periodic() {
       return;  // Add code here to run every loop
    }

    public void setSpeed(double speed) {
       leftMotor.set(speed);
        // Add code here to set the speed of the climber
    }

} 
