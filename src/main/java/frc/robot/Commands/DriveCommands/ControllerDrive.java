package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode.DriveMode;

public class ControllerDrive extends InstantCommand{

    public ControllerDrive() {
        super(() -> {
            Robot.robotMode.setDriveMode(DriveMode.FieldCentric);
            Robot.robotMode.setSwerveControl(() -> Robot.joystick.getLeftY(), () -> Robot.joystick.getLeftX(), () -> Robot.joystick.getRightX());
        });
    }
    
}
