package frc.robot.Commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.Commands.manipulator.SetManipulatorWheelSpeed;

public class PrepL2Auton extends SequentialCommandGroup{
    
    public PrepL2Auton() {
        addCommands(
            new SetManipulatorWheelSpeed(-0.05),
            new SetArmAngle(-0.44),
            new SetElevatorPosition(0.02),
            new SetIntakePosition(0.32)
        );
    }

}
