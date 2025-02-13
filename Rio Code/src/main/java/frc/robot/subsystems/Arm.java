package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Arm extends SubsystemBase {

    private final TalonFX armMotor;
    private final CANcoder armEncoder;
    private final TalonFXConfiguration ArmConfig;
    private final CANcoderConfiguration encoderConfig;
    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;

    private double targetAngle;

    public Arm() {
        // Motor basic setup
        armMotor = new TalonFX(Constants.Arm.ArmMotorID);
        armEncoder = new CANcoder(Constants.Arm.ArmEncoderID);
        ArmConfig = new TalonFXConfiguration();
        encoderConfig = new CANcoderConfiguration();
        ArmConfig.Feedback.SensorToMechanismRatio = Constants.Arm.conversionFactor;
        ArmConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        ArmConfig.Feedback.FeedbackRemoteSensorID = Constants.Arm.ArmEncoderID;
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
      Logger.log("Arm/Absolute Position", Units.rotationsToDegrees(armEncoder.getPosition().getValueAsDouble()));
    }

    public void setSpeed(double speed) {
        armMotor.set(speed);
    }

    public void setPositon(double angle) {
      targetAngle = angle;
      if (angle < Constants.Arm.angleMax && angle > Constants.Arm.angleMin) {
        armMotor.setControl(new MotionMagicVoltage(Units.degreesToRotations(angle)));
      } else {
          SignalLogger.writeString("Arm position out of bounds", String.valueOf(angle));
      }
    }

    public boolean atPosition() {
      return (targetAngle - getAngle() < 0.1) && (targetAngle - getAngle() > -0.1);
    }

    public double getAngle() {
      return Units.rotationsToDegrees(armEncoder.getPosition().getValueAsDouble());
    }
  }