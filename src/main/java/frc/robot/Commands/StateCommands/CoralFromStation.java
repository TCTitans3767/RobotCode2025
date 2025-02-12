package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.Commands.GeneralControlCommands.SetArmAngle;
import frc.robot.Commands.GeneralControlCommands.SetElevatorPosition;

public class CoralFromStation extends SequentialCommandGroup{

    public CoralFromStation() {
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorPosition(0.5),
                new SetArmAngle(90)
                // new SetIntakePosition(),
                // new SetClimbPosition(),
                // new SetManipulatorSpeed(),
                // new SetDrivertrainTarget()
            ),
            new WaitUntilCommand(() -> Robot.manipulator.hasGamePiece())
        );
        addRequirements(Robot.robotMode);
    }
    
}
