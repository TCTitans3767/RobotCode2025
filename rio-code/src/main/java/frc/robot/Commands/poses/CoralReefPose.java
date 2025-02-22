package frc.robot.Commands.poses;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ButtonBox;
import frc.robot.DashboardButtonBox;
import frc.robot.ReefLevel;
import frc.robot.Robot;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.RobotMode;

public class CoralReefPose extends SequentialCommandGroup{

    public class L1 extends ParallelCommandGroup{
        public L1() {
            addCommands(
                new SetArmAngle(0.02),
                new SetElevatorPosition(0.34)
            );
        }
    }

    public class L2 extends SequentialCommandGroup{
        public L2() {
            addCommands(
                new SetArmAngle(-0.44),
                new SetElevatorPosition(0.02)
            );
        }
    }

    public class L3 extends ParallelCommandGroup{
        public L3() {
            addCommands(
                new SetArmAngle(-0.43),
                new SetElevatorPosition(0.44)
            );
        }
    }

    public class L4 extends ParallelCommandGroup{
        public L4() {
            addCommands(
                new SetArmAngle(-0.45),
                new SetElevatorPosition(1.06)
            );
        }
    }

    private Map<String, Command> commandMap = new HashMap<String, Command>();
    
    public CoralReefPose() {

        commandMap.put("1", new L1());
        commandMap.put("2", new L2());
        commandMap.put("3", new L3());
        commandMap.put("4", new L4());

        addCommands(
            new SelectCommand<String>(commandMap, DashboardButtonBox::getSelectedLevelString),
            new InstantCommand(() -> {Robot.manipulator.setSpeed(0);}),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(RobotMode.coralReef);})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
