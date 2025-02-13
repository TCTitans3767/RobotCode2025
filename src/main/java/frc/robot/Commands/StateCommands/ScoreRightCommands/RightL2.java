package frc.robot.Commands.StateCommands.ScoreRightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.GeneralControlCommands.SetArmAngle;
import frc.robot.Commands.GeneralControlCommands.SetElevatorPosition;

public class RightL2 extends SequentialCommandGroup{
    
    public RightL2() {
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
