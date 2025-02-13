package frc.robot.Commands.DriveCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.RobotMode.DriveMode;

public class ControllerDrive extends InstantCommand{

    public ControllerDrive() {
        super(() -> {
            Robot.robotMode.setDriveMode(DriveMode.TeleopDrive);
            Robot.robotMode.setSwerveControl(() -> Robot.joystick.getLeftY(), () -> Robot.joystick.getLeftX(), () -> Robot.joystick.getRightX());
        });
    }
    
}
