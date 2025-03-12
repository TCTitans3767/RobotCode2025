package frc.robot.Commands.drive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode.DriveMode;

public class SlowControllerDrive extends InstantCommand{

    public SlowControllerDrive() {
        super(() -> {
            Robot.robotMode.setDriveMode(DriveMode.TeleopDrive);
            Robot.robotMode.setSwerveControl(Robot.getAlliance() == Alliance.Blue ? () -> Robot.joystick.getLeftY() * 0.5 : () -> -Robot.joystick.getLeftY() * 0.5, Robot.getAlliance() == Alliance.Blue ? () -> Robot.joystick.getLeftX() * 0.5 : () -> -Robot.joystick.getLeftX() * 0.5, () -> Robot.joystick.getRightX() * 0.5);
        }, Robot.drivetrain);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}
