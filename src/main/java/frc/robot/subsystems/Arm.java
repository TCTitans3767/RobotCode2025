package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Arm extends SubsystemBase {

  private final TalonFX armMotor;
    private final TalonFXConfiguration ArmConfig;
    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;

    public Arm() {
        // Motor basic setup
        armMotor = new TalonFX(Constants.Arm.ArmMotorID);
        ArmConfig = new TalonFXConfiguration();
        ArmConfig.Feedback.SensorToMechanismRatio = Constants.Arm.conversionFactor;
        ArmConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // Slot 0 PID setup
        slot0Config = new Slot0Configs();
        slot0Config.kP = Constants.Arm.kP;
        slot0Config.kI = Constants.Arm.kI;
        slot0Config.kD = Constants.Arm.kD;
        slot0Config.kG = Constants.Arm.kG;
        slot0Config.kV = Constants.Arm.kV;
        slot0Config.kS = Constants.Arm.kS;
        slot0Config.GravityType = GravityTypeValue.Arm_Cosine;

         // Motion Magic setup
         motionMagicConfig = new MotionMagicConfigs();
         motionMagicConfig.MotionMagicCruiseVelocity = Constants.Arm.maxVelocity;
         motionMagicConfig.MotionMagicAcceleration = Constants.Arm.maxAcceleration;

           // Set the configurations
        armMotor.getConfigurator().apply(ArmConfig);
        armMotor.getConfigurator().apply(slot0Config);
        armMotor.getConfigurator().apply(motionMagicConfig);
        armMotor.setNeutralMode(NeutralModeValue.Brake);

    }
    
    @Override
    public void periodic() {
      return; // This is a method that is called periodically <- im so smart
    }

    public void setSpeed(double speed) {
        armMotor.set(speed);
    }

    public void setPositon(double position) {
      if (position < Constants.Arm.angleMax && position > Constants.Arm.angleMin) {
        armMotor.setControl(new MotionMagicVoltage(position));
      } else {
          SignalLogger.writeString("Arm position out of bounds", String.valueOf(position));
      }
    }
  }