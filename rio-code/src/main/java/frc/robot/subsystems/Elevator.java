package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Logger;
import frc.robot.utils.Utils;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
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
    private final MotionMagicVoltage motionMagicRequest;

    private double targetRotations;

    public Elevator() {
        // Motor basic setup
        leftMotor = new TalonFX(Constants.Elevator.leftMotorID);
        leftConfig = new TalonFXConfiguration();
        leftConfig.Feedback.SensorToMechanismRatio = Constants.Elevator.conversionFactor;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Elevator.metersMax * Constants.Elevator.RotationsPerMeter;
        leftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Elevator.metersMin * Constants.Elevator.RotationsPerMeter;
        leftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        
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
        leftMotor.setNeutralMode(NeutralModeValue.Brake);

        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor = new TalonFX(Constants.Elevator.rightMotorID);
        rightMotor.getConfigurator().apply(rightConfig);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setControl(new Follower(Constants.Elevator.leftMotorID, true));

        motionMagicRequest = new MotionMagicVoltage(0);

    }

    @Override
    public void periodic() {
        // Logger.log("Elevator/Set Speed", leftMotor.get());
        // Logger.log("Elevator/Speed", leftMotor.getVelocity().getValueAsDouble());
        // Logger.log("Elevator/height meters", leftMotor.getPosition().getValueAsDouble() / Constants.Elevator.RotationsPerMeter);
        // Logger.log("Elevator/velocity", leftMotor.getVelocity().getValueAsDouble());
        // Logger.log("Elevator/at position", isAtPosition());

        // if (leftMotor.getPosition().getValueAsDouble() <= 0.5 || leftMotor.getPosition().getValueAsDouble() >= 99999 ) {
        //     leftMotor.set(0);
        //     rightMotor.setControl(new Follower(Constants.Elevator.leftMotorID, true));
        // }

        if (!isAtPosition()) {
            logElevatorPosition();
        }
        
    }

    public void setSpeed(double speed) {
        Logger.log("Elevator/target speed", speed);
        leftMotor.set(speed);
    }

    public void setPosition(double meters) {
        if (meters < Constants.Elevator.metersMin || meters > Constants.Elevator.metersMax ) {
            Logger.logLimitError("Invalid Elevator Height: " + meters);
            return;
        }

        targetRotations = meters * Constants.Elevator.RotationsPerMeter;
        leftMotor.setControl(motionMagicRequest.withPosition(meters * Constants.Elevator.RotationsPerMeter));

        Logger.logElevatorSetHeight(meters);
        if (!isAtPosition()) {
            logElevatorPosition();
        }
    }

    public boolean isAtPosition() {
        return MathUtil.isNear(targetRotations, leftMotor.getPosition().getValueAsDouble(), Constants.Elevator.errorTolerance);
    }

    public boolean isAtPosition(double meters) {
        return MathUtil.isNear(meters * Constants.Elevator.RotationsPerMeter, leftMotor.getPosition().getValueAsDouble(), Constants.Elevator.errorTolerance);
    }

    public double getHeightMeters() {
        return leftMotor.getPosition().getValueAsDouble() / Constants.Elevator.RotationsPerMeter;
    }
    
    public void resetEncoder() {
        leftMotor.setPosition(0);
    }

    public void logElevatorPosition() {
        Logger.logElevatorHeight(leftMotor.getPosition().getValueAsDouble() / Constants.Elevator.RotationsPerMeter);
        Logger.logElevatorMotorSpeed(leftMotor.get());
        Logger.logElevatorMotorVelocity(leftMotor.getVelocity().getValueAsDouble());
        Logger.logElevatorAtPosition(isAtPosition());
    }

    public double getMotorTorque() {
        return leftMotor.getTorqueCurrent().getValueAsDouble();
    }

    public void disableSoftwareLimits() {
        leftMotor.getConfigurator().refresh(new SoftwareLimitSwitchConfigs()
                                            .withReverseSoftLimitEnable(false)
                                            .withForwardSoftLimitEnable(false)
                                            .withForwardSoftLimitThreshold(Constants.Elevator.metersMax)
                                            .withReverseSoftLimitThreshold(Constants.Elevator.metersMin)
                                        );
    }

    public void enableSoftwareLimits() {
        leftMotor.getConfigurator().refresh(new SoftwareLimitSwitchConfigs()
                                            .withReverseSoftLimitEnable(true)
                                            .withForwardSoftLimitEnable(true)
                                            .withForwardSoftLimitThreshold(Constants.Elevator.metersMax)
                                            .withReverseSoftLimitThreshold(Constants.Elevator.metersMin)
                                        );
    }

    public double getSpeed() {
        return leftMotor.get();
    }

    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble() / Constants.Elevator.RotationsPerMeter;
    }
}
