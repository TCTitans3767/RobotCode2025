package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.AutonCommands.CoralStationAuton;
import frc.robot.Commands.AutonCommands.CoralStationAutonCommand;
import frc.robot.Commands.AutonCommands.GroundIntakeAuton;
import frc.robot.Commands.AutonCommands.PrepL2Auton;
import frc.robot.Commands.AutonCommands.PrepL4Auton;
import frc.robot.Commands.AutonCommands.ScoreAuto;
import frc.robot.Commands.AutonCommands.ScoreL1;
import frc.robot.Commands.AutonCommands.ScoreL1AutonCommand;
import frc.robot.Commands.Intake.SetIntakePosition;
import frc.robot.Commands.Intake.SetIntakeWheelSpeed;
import frc.robot.Commands.arm.SetArmAngle;
import frc.robot.Commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.RobotMode;

public class Autos {

    public static Command lolipopAuto(AutoFactory factory) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> Robot.drivetrain.resetPose(Choreo.loadTrajectory("Wall Start To A4").get().getInitialPose(Robot.getAlliance() == Alliance.Red).get())),
            // new InstantCommand(() -> Robot.limelight.turnOnAprilTags()),
            new ParallelCommandGroup(
                factory.trajectoryCmd("Wall Start To A4"),
                new SetIntakePosition(0.32),
                new SequentialCommandGroup(
                    new SetArmAngle(-0.128),
                    new SetElevatorPosition(0.5)
                )
            ),
            new PrepL4Auton(),
            new WaitCommand(0.1),
            // new InstantCommand(() -> Robot.limelight.turnOffAprilTags()),
            factory.trajectoryCmd("A4 Lineup"),
            new ScoreAuto(),
            new WaitCommand(0.1),
            new SetIntakeWheelSpeed(0.5),
            new GroundIntakeAuton(),
            factory.trajectoryCmd("A4 To Lolipop 2"),
            new WaitUntilCommand(TriggerBoard::isCoralInManipulator),
            factory.trajectoryCmd("Lolipop 2 To A2"),
            new PrepL2Auton(),
            factory.trajectoryCmd("A Lineup"),
            new ScoreAuto()
        );
    }

    public static Command L1LeftCommandGroup(AutoFactory factory) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> Robot.drivetrain.resetPose(Choreo.loadTrajectory("Score L1 Left").get().getInitialPose(Robot.getAlliance() == Alliance.Red).get())),
            new ParallelCommandGroup(
                factory.trajectoryCmd("Score L1 Left"),
                new SetIntakePosition(0.20)
            ),
            new ScoreL1AutonCommand(),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                factory.trajectoryCmd("L1 Left To Coral Station"),
                new CoralStationAutonCommand()
            ),
            new WaitCommand(0.3),
            factory.trajectoryCmd("Left Coral Station To I4"),
            new PrepL4Auton(),
            factory.trajectoryCmd("L4 Lineup"),
            new InstantCommand(() -> {Robot.robotMode.setCurrentMode(RobotMode.scoreCoralPose);})
        );
    }

    public static AutoRoutine L1Left(AutoFactory factory) {
        final AutoRoutine routine = factory.newRoutine("ScoreL1 Left");

        final frc.robot.Commands.AutonCommands.ScoreL1 scoreL1 = new frc.robot.Commands.AutonCommands.ScoreL1(routine.loop());
        final CoralStationAuton coralStationAuton = new CoralStationAuton(routine.loop());

        final AutoTrajectory scoreL1Path = routine.trajectory("Score L1 Left");

        routine.active().onTrue(new InstantCommand(
                () -> {
                    Robot.getDrivetrain().resetPose(scoreL1Path.getInitialPose().get());
                    Robot.intake.setPivotPosition(0.18);
                }
            )
        ).onTrue(scoreL1Path.cmd());

        scoreL1Path.atTime("Score L1").onTrue(scoreL1.cmd());

        scoreL1Path.atTime("Prep Coral Station").onTrue(coralStationAuton.cmd());

        return routine;

    }

    public static AutoRoutine L1RightNoExtra(AutoFactory factory) {
        final AutoRoutine routine = factory.newRoutine("ScoreL1 Right No Extra");

        final frc.robot.Commands.AutonCommands.ScoreL1 scoreL1 = new frc.robot.Commands.AutonCommands.ScoreL1(routine.loop());
        final CoralStationAuton coralStationAuton = new CoralStationAuton(routine.loop());

        final AutoTrajectory scoreL1Path = routine.trajectory("ScoreL1 Right No Extra");

        routine.active().onTrue(new InstantCommand(
                () -> {
                    Robot.getDrivetrain().resetPose(scoreL1Path.getInitialPose().get());
                    Robot.intake.setPivotPosition(0.18);
                }
            )
        ).onTrue(scoreL1Path.cmd());

        scoreL1Path.atTime("ScoreL1").onTrue(scoreL1.cmd());

        return routine;

    }

    public static AutoRoutine L1Right(AutoFactory factory) {
        final AutoRoutine routine = factory.newRoutine("ScoreL1 Right");

        final frc.robot.Commands.AutonCommands.ScoreL1 scoreL1 = new frc.robot.Commands.AutonCommands.ScoreL1(routine.loop());
        final CoralStationAuton coralStationAuton = new CoralStationAuton(routine.loop());

        final AutoTrajectory scoreL1Path = routine.trajectory("ScoreL1 Right");
        final AutoTrajectory scoreL4Path = routine.trajectory("Right Coral Station To B4");

        routine.active().onTrue(new InstantCommand(
                () -> {
                    Robot.getDrivetrain().resetPose(scoreL1Path.getInitialPose().get());
                    Robot.intake.setPivotPosition(0.18);
                }
            )
        ).onTrue(scoreL1Path.cmd());

        scoreL1Path.atTime("ScoreL1").onTrue(scoreL1.cmd());

        scoreL1Path.atTime("Coral Station").onTrue(coralStationAuton.cmd());

        coralStationAuton.done().onTrue(scoreL4Path.cmd());

        return routine;

    }

    public static AutoRoutine L1B4(AutoFactory factory) {
        final AutoRoutine routine = factory.newRoutine("L1B4");

        final CoralStationAuton coralStation = new CoralStationAuton(routine.loop());

        final AutoTrajectory StartToCoralStation = routine.trajectory("Start To Coral Station");
        final AutoTrajectory CoralStationToB4 = routine.trajectory("Coral Station To B4");

        routine.active().onTrue(new InstantCommand(
                () -> Robot.getDrivetrain().resetPose(StartToCoralStation.getInitialPose().get())
            )
        ).onTrue(StartToCoralStation.cmd());

        StartToCoralStation.atTime("Prepare Coral Station").onTrue(coralStation.cmd());

        coralStation.done().onTrue(CoralStationToB4.cmd());

        return routine;
    }
    
    public static AutoRoutine test(AutoFactory factory) {
        final AutoRoutine routine = factory.newRoutine("test");

        final AutoTrajectory trajectory = routine.trajectory("Test Path");

        routine.active().onTrue(new InstantCommand(
                () -> Robot.getDrivetrain().resetPose(trajectory.getInitialPose().get())
            )
        )
        .onTrue(trajectory.cmd());

        trajectory.done().onTrue(new InstantCommand(() -> Robot.getDrivetrain().setControl(new SwerveRequest.SwerveDriveBrake())));

        return routine;
    }

}
