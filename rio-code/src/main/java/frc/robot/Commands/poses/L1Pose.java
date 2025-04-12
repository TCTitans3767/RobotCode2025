package frc.robot.Commands.poses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.Intake.SetIntakeWheelSpeed;
import frc.robot.subsystems.RobotMode;

public class L1Pose extends SequentialCommandGroup{

    public L1Pose() {
        addCommands(
            new SetIntakePosition(0.2),
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(RobotMode.L1))
        );
    }

}
