package frc.robot.Commands.StateCommands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Commands.DriveCommands.TeleopDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.RobotMode;

public class Idle extends Command{

    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Manipulator manipulator;
    private final Climber climber;
    private final Supplier<SwerveRequest> request;
    private final RobotMode robotController;

    private final TeleopDrive teleopDrive;

    public Idle(RobotMode robotController, Supplier<SwerveRequest> request) {
        drivetrain = Robot.drivetrain;
        elevator = Robot.elevator;
        manipulator = Robot.manipulator;
        climber = Robot.climber;
        this.request = request;
        this.robotController = robotController;

        teleopDrive = new TeleopDrive(request, drivetrain);
        addRequirements(robotController);
    }

    @Override
    public void initialize() {
        teleopDrive.schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        teleopDrive.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
