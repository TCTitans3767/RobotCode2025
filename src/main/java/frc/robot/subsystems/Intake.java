package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Intake extends SubsystemBase{
    private final TalonFX wheelMotor, pivotMotor;
    private final TalonFXConfiguration wheelConfig, pivotConfig;

    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;

    public Intake() {
        // Motor basic setup
        wheelMotor = new TalonFX(Constants.Intake.wheelMotorID);
        wheelConfig = new TalonFXConfiguration();
        wheelConfig.Feedback.SensorToMechanismRatio = Constants.Intake.conversionFactor;
        wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotMotor = new TalonFX(Constants.Intake.pivotMotorID);
        pivotConfig = new TalonFXConfiguration();
         
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
        wheelMotor.getConfigurator().apply(slot0Config);
        wheelMotor.getConfigurator().apply(motionMagicConfig);
        wheelMotor.setNeutralMode(NeutralModeValue.Brake);

        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    public void setSpeed(double speed) {
        wheelMotor.set(speed);
    
     }

     public void setPosition(double position) {
        if (position < Constants.Intake.angleMax && position > Constants.Intake.angleMin) {
            pivotMotor.setControl(new MotionMagicVoltage(position));
        } else {
            SignalLogger.writeString("Intake position out of bounds", String.valueOf(position));
        }
    }
}
