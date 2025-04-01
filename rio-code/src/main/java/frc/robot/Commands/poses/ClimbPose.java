package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.Commands.modes.Climb;
import frc.robot.subsystems.RobotMode;

public class ClimbPose extends SequentialCommandGroup{
    
    public ClimbPose() {

        addCommands(
            new SetArmAngle(0.35),
            new SetElevatorPosition(0.021),
            new SetIntakePosition(0.15),
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(RobotMode.deployClimberPose))
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
