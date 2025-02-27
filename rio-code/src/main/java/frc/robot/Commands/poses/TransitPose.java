package frc.robot.Commands.poses;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.drive.ControllerDrive;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.RobotMode;

public class TransitPose extends SequentialCommandGroup{

    private enum gamePieceState {
        coralInManipulator,
        algaeInIntake,
        coralInManipulatorAndAlgaeInIntake,
        noGamePiece
    }

    private class AlgaeInIntakeTransit extends ParallelCommandGroup {
        public AlgaeInIntakeTransit() {
            addCommands(
                new SetElevatorPosition(0.5),
                new SetArmAngle(-0.128),
                new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);})
            );
        }
    }

    private class CoralInManipulatorAndAlgaeInIntakeTransit extends ParallelCommandGroup {
        public CoralInManipulatorAndAlgaeInIntakeTransit() {
            addCommands(
                new SetElevatorPosition(0.5),
                new SetArmAngle(-0.128),
                new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);})
            );
        }
    }

    private class CoralInManipulatorTransit extends SequentialCommandGroup {
        public CoralInManipulatorTransit() {
            addCommands(
                new SetArmAngle(-0.128),
                new SetElevatorPosition(0.5),
                new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);})
            );
        }
    }

    private class NoGamePieceTransit extends ParallelCommandGroup {
        public NoGamePieceTransit() {
            addCommands(
                new SetElevatorPosition(0.5),
                new SetArmAngle(0.128),
                new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);})
            );
        }
    }

    private Map<gamePieceState, Command> commandMap = new HashMap<gamePieceState, Command>();

    public TransitPose() {

        commandMap.put(gamePieceState.algaeInIntake, new AlgaeInIntakeTransit());
        commandMap.put(gamePieceState.coralInManipulatorAndAlgaeInIntake, new CoralInManipulatorAndAlgaeInIntakeTransit());
        commandMap.put(gamePieceState.coralInManipulator, new CoralInManipulatorTransit());
        commandMap.put(gamePieceState.noGamePiece, new NoGamePieceTransit());

        addCommands(
            new SelectCommand<gamePieceState>(commandMap, TransitPose::currentGamePieceState),
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(RobotMode.transit))
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    public static gamePieceState currentGamePieceState() {
        if (TriggerBoard.isCoralInManipulator() && !TriggerBoard.isAlgaeInIntake()) {
            return gamePieceState.coralInManipulator;
        } else if (TriggerBoard.isCoralInManipulator() && TriggerBoard.isAlgaeInIntake()) {
            return gamePieceState.coralInManipulatorAndAlgaeInIntake;
        } else if (TriggerBoard.isAlgaeInIntake() && !TriggerBoard.isCoralInManipulator()) {
            return gamePieceState.algaeInIntake;
        } else {
            return gamePieceState.noGamePiece;
        }
    }

}
