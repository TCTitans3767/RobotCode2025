package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.AutonCommands.CoralStationAuton;

public class Autos {

    public static Command L1LeftCommandGroup() {
        return new SequentialCommandGroup(
            
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

        scoreL1Path.atTime("ScoreL1").onTrue(scoreL1.cmd());
        
        scoreL1Path.atTime("Coral Station").onTrue(coralStationAuton.cmd());

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

        routine.active().onTrue(new InstantCommand(
                () -> {
                    Robot.getDrivetrain().resetPose(scoreL1Path.getInitialPose().get());
                    Robot.intake.setPivotPosition(0.18);
                }
            )
        ).onTrue(scoreL1Path.cmd());

        scoreL1Path.atTime("ScoreL1").onTrue(scoreL1.cmd());

        scoreL1Path.atTime("Coral Station").onTrue(coralStationAuton.cmd());

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
