package frc.robot.Commands.StateCommands.PrepCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.GeneralControlCommands.SetArmAngle;
import frc.robot.Commands.GeneralControlCommands.SetElevatorPosition;

public class PrepL1 extends SequentialCommandGroup{
    
    public PrepL1() {
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorPosition(0),
                new SetArmAngle(0)//,
                // new SetIntakePosition(),
                // new SetClimbPosition(),
                // new SetManipulatorSpeed(),
                // new SetDrivertrainTarget()
            ),
            new WaitUntilCommand(() -> true)
        );
    }
}
