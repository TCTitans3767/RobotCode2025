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


public class Elevator extends SubsystemBase{
    private final TalonFX rightMotor, leftMotor;
    private final TalonFXConfiguration leftConfig, rightConfig;

    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;

    public Elevator() {
        // Motor basic setup
        leftMotor = new TalonFX(Constants.Elevator.leftMotorID);
        leftConfig = new TalonFXConfiguration();
        leftConfig.Feedback.SensorToMechanismRatio = Constants.Elevator.conversionFactor;
        
        // Slot 0 PID setup
        slot0Config = new Slot0Configs();
        slot0Config.kP = Constants.Elevator.kP;
        slot0Config.kI = Constants.Elevator.kI;
        slot0Config.kD = Constants.Elevator.kD;
        slot0Config.kG = Constants.Elevator.kG;
        slot0Config.kV = Constants.Elevator.kV;
        slot0Config.kS = Constants.Elevator.kS;
        slot0Config.GravityType = GravityTypeValue.Elevator_Static;

        // Motion Magic setup
        motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.Elevator.maxVelocity;
        motionMagicConfig.MotionMagicAcceleration = Constants.Elevator.maxAcceleration;

        // Set the configurations
        leftMotor.getConfigurator().apply(leftConfig);
        leftMotor.getConfigurator().apply(slot0Config);
        leftMotor.getConfigurator().apply(motionMagicConfig);
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.setNeutralMode(NeutralModeValue.Brake);

        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor = new TalonFX(Constants.Elevator.rightMotorID);
        rightMotor.getConfigurator().apply(rightConfig);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    @Override
    public void periodic() {
        return; // This is a method that is called periodically <- im so smart
    }

    public void setSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.setControl(new Follower(Constants.Elevator.leftMotorID, true));
    }

    
    
}
