package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Logger;

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

    private double targetRotations = 0;

    public Climber() {
        // Initialize the motors
        leftMotor = new TalonFX(Constants.Climber.leftMotorID); 
        leftConfig = new TalonFXConfiguration();
        leftConfig.Feedback.SensorToMechanismRatio = Constants.Climber.conversonFactor;
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        slot0Config = new Slot0Configs();
        slot0Config.kP = 80;
        slot0Config.kI = 0;
        slot0Config.kD = 0;
        slot0Config.GravityType = GravityTypeValue.Elevator_Static;
        slot0Config.kG = 0; // Gravity
        slot0Config.kS = 0;


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
        Logger.log("Climber/current Rotations", leftMotor.getPosition().getValueAsDouble());
        return;  // Add code here to run every loop
    }

    public TalonFX getLeftMotor() {
        return leftMotor;
    }

    public TalonFX getRightMotor() {
        return rightMotor;
    }

    public void setSpeed(double speed) {
        Logger.log("Climber/target speed", speed);
        leftMotor.set(speed);
        // Add code here to set the speed of the climber
    }

    public void setRotations(double rotations) {
        targetRotations = rotations;
        leftMotor.setControl(new MotionMagicVoltage(rotations));
    }

    public boolean isAtPosition() {
        return MathUtil.isNear(targetRotations, leftMotor.getPosition().getValueAsDouble(), 0.1);
    }

} 
