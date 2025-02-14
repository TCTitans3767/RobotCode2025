package frc.robot.Commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.Commands.DriveCommands.AlignWithCoralStation;

public class CoralStationActive extends SequentialCommandGroup{
    
    public CoralStationActive() {

        until(() -> !Robot.drivetrain.closeToCoralStation());

        addCommands(
            new WaitUntilCommand(Robot.joystick.leftTrigger()),
            new ParallelRaceGroup(
                new AlignWithCoralStation(),
                new WaitUntilCommand(() -> Robot.manipulator.hasGamePiece()),
                new WaitUntilCommand(Robot.joystick.leftTrigger().negate())
            )
        );

        addRequirements(Robot.robotMode);
    }

}
