package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Logger;

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
import com.ctre.phoenix6.signals.SensorDirectionValue;


public class Arm extends SubsystemBase {

    private final TalonFX armMotor;
    private final CANcoder armEncoder;
    private final TalonFXConfiguration armMotorConfig;
    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;
    private final CANcoderConfiguration armEncoderConfig;

    private double targetRotations;

    public Arm() {
        // Motor basic setup
        armMotor = new TalonFX(Constants.Arm.ArmMotorID);
        armEncoder = new CANcoder(Constants.Arm.ArmEncoderID);
        armMotorConfig = new TalonFXConfiguration();
        armMotorConfig.Feedback.SensorToMechanismRatio = Constants.Arm.conversionFactor;
        armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armMotorConfig.Feedback.FeedbackRemoteSensorID = Constants.Arm.ArmEncoderID;
        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.rotationsMax;
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.rotationsMin;
        armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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

        armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        armEncoder.getConfigurator().apply(armEncoderConfig);

        // Set the configurations
        armMotor.getConfigurator().apply(armMotorConfig);
        armMotor.getConfigurator().apply(slot0Config);
        armMotor.getConfigurator().apply(motionMagicConfig);
        armMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    @Override
    public void periodic() {
      logArmPosition();
    }

    public TalonFX getArmMotor() {
        return this.armMotor;
    }

    public void setSpeed(double speed) {
      armMotor.set(speed);
    }

    public void setPosition(double rotations) {
      if (rotations < Constants.Arm.rotationsMin || rotations > Constants.Arm.rotationsMax) {
        Logger.logLimitError("Invalid Arm position: " + rotations);
        return;
      }

      targetRotations = rotations;
      armMotor.setControl(new MotionMagicVoltage(rotations));

      Logger.logArmSetPosition(rotations);
      if (!isAtPosition()) {
        logArmPosition();
      }
    }

    public boolean isAtPosition() {
      return MathUtil.isNear(targetRotations, armMotor.getPosition().getValueAsDouble(), Constants.Arm.errorTolerance);
    }

    public double getMotionMagicError() {
      return armMotor.getClosedLoopError().getValueAsDouble();
    }

    public double getPosition() {
      return armMotor.getPosition().getValueAsDouble();
    }

    private void logArmPosition() {
      Logger.logArmAbsolute(armEncoder.getAbsolutePosition().getValueAsDouble());
      Logger.logArmMotorPosition(armMotor.getPosition().getValueAsDouble());
      Logger.logArmMotorSpeed(armMotor.get());
      Logger.logArmMotorVelocity(armMotor.getVelocity().getValueAsDouble());
      Logger.logArmAtPosition(isAtPosition());
    }

    public boolean isNear(double position) {
      return MathUtil.isNear(position, armMotor.getPosition().getValueAsDouble(), Constants.Arm.errorTolerance);
    }
  }
