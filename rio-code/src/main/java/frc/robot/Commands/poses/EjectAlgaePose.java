package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.Intake.SetIntakeWheelSpeed;
import frc.robot.subsystems.RobotMode;

public class EjectAlgaePose extends SequentialCommandGroup{
    
    public EjectAlgaePose() {

        addCommands(
            new SetIntakeWheelSpeed(-0.7),
            new InstantCommand(() -> {
                Robot.intake.setWheelSpeed(0);
            }),
            new WaitCommand(0.2),
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(RobotMode.transitPose))
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
