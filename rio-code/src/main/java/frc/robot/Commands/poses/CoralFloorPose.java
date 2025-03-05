package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.Intake.SetIntakeWheelSpeed;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.Commands.manipulator.SetManipulatorWheelSpeed;
import frc.robot.subsystems.RobotMode;

public class CoralFloorPose extends SequentialCommandGroup{
    
    public CoralFloorPose() {

        addCommands(
            new SetIntakePosition(-0.11),
            new ParallelCommandGroup(
                new SetArmAngle(-0.03),
                new SetElevatorPosition(0.03).withTimeout(2),
                new SetManipulatorWheelSpeed(-0.3)
            ),
            new SetIntakeWheelSpeed(0.5),
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(RobotMode.coralFloor))
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
