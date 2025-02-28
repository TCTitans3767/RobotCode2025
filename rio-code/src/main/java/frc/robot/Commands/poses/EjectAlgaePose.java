package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.Intake.SetIntakeWheelSpeed;
import frc.robot.subsystems.RobotMode;

public class EjectAlgaePose extends SequentialCommandGroup{
    
    public EjectAlgaePose() {

        addCommands(
            new SetIntakePosition(0.08),
            new WaitUntilCommand(TriggerBoard::isAlgaeButtonPressed),
            new SetIntakeWheelSpeed(-0.5),
            new InstantCommand(() -> {
                Robot.intake.setWheelSpeed(0);
                Robot.robotMode.setCurrentMode(RobotMode.transitPose);
            })
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
