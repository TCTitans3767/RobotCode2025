package frc.robot.Commands.AutonCommands;

import java.io.SequenceInputStream;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Commands.Intake.SetIntakePivotSpeed;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.Commands.elevator.SetElevatorSpeed;
import frc.robot.Commands.manipulator.SetManipulatorWheelSpeed;

public class GroundIntakeAuton extends ParallelCommandGroup{
    
    public GroundIntakeAuton() {
        addCommands(
            new SetIntakePosition(-0.11),
            new SetElevatorPosition(0.025),
            new SetManipulatorWheelSpeed(-0.2),
            new SetArmAngle(-0.03)
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);

    }

}
