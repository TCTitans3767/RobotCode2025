package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Commands.DriveCommands.ControllerDrive;
import frc.robot.Commands.GeneralControlCommands.SetElevatorPosition;
import frc.robot.utils.Utils;

public class Idle extends Command{

    // private final TeleopDrive teleopDrive;

    public Idle() {
        addRequirements(Robot.robotMode);
    }

    @Override
    public void initialize() {
        // new SetElevatorPosition(0.25);
        new ControllerDrive();
    }
    
}
