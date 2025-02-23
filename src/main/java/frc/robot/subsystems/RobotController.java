package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class RobotController extends SubsystemBase {

    private final Drivetrain drivetrain;

    public RobotController(Drivetrain drivetrain, Supplier<SwerveRequest> request) {
        this.drivetrain = drivetrain;

        // this.setDefaultCommand(idle);
    }

    @Override
    public void periodic() {

        // Logger.log("RobotController/Active Command", this.getCurrentCommand().getName());

    }

    public void idle() {
        // idle.schedule();
    }

    public void cancelAll() {
        CommandScheduler.getInstance().cancelAll();
    }
    
}
