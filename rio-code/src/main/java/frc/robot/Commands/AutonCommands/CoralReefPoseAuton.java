package frc.robot.Commands.AutonCommands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ButtonBox;
import frc.robot.DashboardButtonBox;
import frc.robot.ReefLevel;
import frc.robot.Robot;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.Commands.manipulator.SetManipulatorWheelSpeed;
import frc.robot.subsystems.RobotMode;

public class CoralReefPoseAuton extends SequentialCommandGroup{

    public class L1 extends SequentialCommandGroup{
        public L1() {
            addCommands(
                new ParallelCommandGroup(
                    new SetManipulatorWheelSpeed(-0.05),
                    new SetArmAngle(0.02),
                    new SetElevatorPosition(0.34)
                )
            );
        }
    }

    public class L2 extends SequentialCommandGroup{
        public L2() {
            addCommands(
                new SetManipulatorWheelSpeed(-0.05),
                new SetElevatorPosition(0.02),
                new SetArmAngle(-0.44),
                new WaitCommand(0.25)
            );
        }
    }

    public class L3 extends SequentialCommandGroup{
        public L3() {
            addCommands(
                new SetManipulatorWheelSpeed(-0.05),
                new SetElevatorPosition(0.44),
                new SetArmAngle(-0.43),
                new WaitCommand(0.25)
            );
        }
    }

    public class L4 extends SequentialCommandGroup{
        public L4() {
            addCommands(
                new SetManipulatorWheelSpeed(-0.05),
                new SetElevatorPosition(0.95),
                new SetArmAngle(-0.5),
                new WaitCommand(0.25)
            );
        }
    }

    private Map<String, Command> commandMap = new HashMap<String, Command>();
    
    public CoralReefPoseAuton(String reefLevel) {

        commandMap.put("1", new L1());
        commandMap.put("2", new L2());
        commandMap.put("3", new L3());
        commandMap.put("4", new L4());

        addCommands(
            new SelectCommand<String>(commandMap, () -> reefLevel),
            new InstantCommand(() -> {Robot.manipulator.setSpeed(0);})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
