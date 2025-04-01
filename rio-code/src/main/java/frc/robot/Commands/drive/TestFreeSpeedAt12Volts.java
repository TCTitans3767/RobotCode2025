package frc.robot.Commands.drive;

import java.util.ArrayList;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class TestFreeSpeedAt12Volts extends Command{

    private Drivetrain drivetrain = Robot.drivetrain;

    private final SwerveModule<TalonFX, TalonFX, CANcoder>[] swerveModules;

    public TestFreeSpeedAt12Volts() {
        swerveModules = drivetrain.getModules();
    }

    @Override
    public void initialize() {
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : swerveModules) {
            module.apply(new ModuleRequest().withState(new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(0)))));
        }
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : swerveModules) {
            module.getDriveMotor().setControl(new VoltageOut(12));
        }
    }

}