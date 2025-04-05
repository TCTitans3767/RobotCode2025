package frc.robot.Commands.poses;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.Intake.SetIntakeWheelPower;
import frc.robot.Commands.Intake.SetIntakeWheelSpeed;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.drive.ControllerDrive;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.Commands.manipulator.SetManipulatorWheelSpeed;
import frc.robot.subsystems.RobotMode;

public class TransitPose extends SequentialCommandGroup{

    private enum gamePieceState {
        coralInManipulator,
        algaeInIntake,
        coralInManipulatorAndAlgaeInIntake,
        noGamePiece
    }

    private class AlgaeInIntakeTransit extends SequentialCommandGroup {
        public AlgaeInIntakeTransit() {
            addCommands(
                new ParallelCommandGroup(
                    new SetIntakeWheelPower(0.6),
                    new SetManipulatorWheelSpeed(0)
                ),
                new SetIntakePosition(0),
                new SetElevatorPosition(0.5),
                new SetArmAngle(-0.122),
                new InstantCommand(() -> {if (!DriverStation.isAutonomousEnabled()) {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);}})
            );
        }
    }

    private class CoralInManipulatorAndAlgaeInIntakeTransit extends SequentialCommandGroup {
        public CoralInManipulatorAndAlgaeInIntakeTransit() {
            addCommands(
                new ParallelCommandGroup(
                    new SetIntakeWheelPower(0.6),
                    new SetManipulatorWheelSpeed(-0.05)
                ),
                new SetArmAngle(-0.122),
                new SetIntakePosition(0.25),
                new SetElevatorPosition(0.5),
                new InstantCommand(() -> {if (!DriverStation.isAutonomousEnabled()) {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);}})
            );
        }
    }

    private class CoralInManipulatorTransit extends SequentialCommandGroup {
        public CoralInManipulatorTransit() {
            addCommands(
                new ParallelCommandGroup(
                    new SetManipulatorWheelSpeed(-0.05),
                    new SetArmAngle(0.2)
                ),
                new ParallelCommandGroup(
                    new SetManipulatorWheelSpeed(0),
                    new InstantCommand(() -> {Robot.intake.resetWheelSpeed();})
                ),
                new SetElevatorPosition(0.02),
                new InstantCommand(() -> {if (!DriverStation.isAutonomousEnabled()) {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);}})
            );
        }
    }

    private class NoGamePieceTransit extends SequentialCommandGroup {
        public NoGamePieceTransit() {
            addCommands(
                new ParallelCommandGroup(
                    new SetManipulatorWheelSpeed(0),
                    new SetArmAngle(-0.378),
                    new InstantCommand(() -> {Robot.intake.resetWheelSpeed();})
                ),
                new ParallelCommandGroup(
                    new SetIntakePosition(Constants.Intake.pivotStowPosition),
                    new SetElevatorPosition(0.5),
                    new SetArmAngle(0.122)
                ),
                new InstantCommand(() -> {if (!DriverStation.isAutonomousEnabled()) {Robot.robotMode.setDriveModeCommand(RobotMode.controllerDrive);}})
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
