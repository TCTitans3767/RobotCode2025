package frc.robot.Commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.Commands.manipulator.SetManipulatorWheelSpeed;

public class PrepL4Auton extends ParallelCommandGroup{
    
    public PrepL4Auton() {
        addCommands(
            new SetManipulatorWheelSpeed(-0.05),
            new SetArmAngle(-0.43),
            new SetElevatorPosition(1.1),
            new SetIntakePosition(0.32)
        );
    }

}
