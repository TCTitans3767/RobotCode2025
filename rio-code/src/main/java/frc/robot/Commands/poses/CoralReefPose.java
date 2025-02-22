package frc.robot.Commands.poses;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ButtonBox;
import frc.robot.ReefLevel;
import frc.robot.Robot;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.RobotMode;

public class CoralReefPose extends SequentialCommandGroup{

    public class L1 extends SequentialCommandGroup{
        public L1() {
            addCommands();
        }
    }

    public class L2 extends SequentialCommandGroup{
        public L2() {
            addCommands(
                new InstantCommand(() -> {Robot.manipulator.setSpeed(-0.1);}),
                new SetArmAngle(-0.42),
                new SetElevatorPosition(0.08)
            );
        }
    }

    public class L3 extends SequentialCommandGroup{
        public L3() {
            addCommands(
                new InstantCommand(() -> {Robot.manipulator.setSpeed(-0.1);}),
                new SetArmAngle(-0.43),
                new SetElevatorPosition(0.44)
            );
        }
    }

    public class L4 extends SequentialCommandGroup{
        public L4() {
            addCommands(
                new InstantCommand(() -> {Robot.manipulator.setSpeed(-0.1);}),
                new SetArmAngle(-0.45),
                new SetElevatorPosition(1.05)
            );
        }
    }

    private Map<ReefLevel, Command> commandMap = new HashMap<ReefLevel, Command>();
    
    public CoralReefPose() {

        commandMap.put(ReefLevel.L1, new L1());
        commandMap.put(ReefLevel.L2, new L2());
        commandMap.put(ReefLevel.L3, new L3());
        commandMap.put(ReefLevel.L4, new L4());


        addCommands(
            new SelectCommand<ReefLevel>(commandMap, ButtonBox::getSelectedLevel),
            new InstantCommand(() -> {Robot.manipulator.setSpeed(0);}),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(RobotMode.coralReef);})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    // @Override
    // public void initialize() {
    //     if (TriggerBoard.isL1Selected()) {

    //     } else if (TriggerBoard.isL2Selected()) {

    //         Robot.manipulator.setSpeed(-0.1);
    //         Robot.elevator.setPosition(0.05);
    //         Robot.arm.setPosition(-0.42);

    //     } else if (TriggerBoard.isL3Selected()) {

    //         Robot.manipulator.setSpeed(-0.1);
    //         Robot.elevator.setPosition(0.44);
    //         Robot.arm.setPosition(-0.43);

    //     } else if (TriggerBoard.isL4Selected()) {

    //         Robot.manipulator.setSpeed(-0.1);
    //         Robot.elevator.setPosition(1.05);
    //         Robot.arm.setPosition(-0.45);

    //     } else {
    //         Robot.joystick.setRumble(RumbleType.kBothRumble, 1);
    //         Robot.joystick.setRumble(RumbleType.kBothRumble, 0);
    //         Robot.robotMode.setCurrentMode(RobotMode.transitPose);
    //     }

    // }

    // @Override
    // public boolean isFinished() {
    //     return Robot.elevator.isAtPosition() && Robot.arm.isAtPosition();
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     Robot.manipulator.setSpeed(0);
    //     Robot.robotMode.setCurrentMode(RobotMode.coralReef);
    // }

}
