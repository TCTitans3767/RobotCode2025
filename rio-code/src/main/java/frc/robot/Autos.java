package frc.robot;

import java.util.logging.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.AnalogTriggerOutput.AnalogTriggerOutputException;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.AutonCommands.AlignWithLeftReefAuton;
import frc.robot.Commands.AutonCommands.AlignWithRightReefAuton;
import frc.robot.Commands.AutonCommands.CoralReefAlignPoseAuton;
import frc.robot.Commands.AutonCommands.CoralReefPoseAuton;
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
import frc.robot.Commands.modes.CoralReef;
import frc.robot.Commands.poses.TransitPose;
import frc.robot.subsystems.RobotMode;
import frc.robot.utils.Utils.ReefPosition;

public class Autos {

    public static Command J4_K4_L4_CoralStation(AutoFactory factory) {

        try {
            PathPlannerPath rightReefStart_J = PathPlannerPath.fromPathFile("LeftReefStart-J");
            PathPlannerPath J_CoralStation = PathPlannerPath.fromPathFile("J-CoralStation");
            PathPlannerPath CoralStation_K = PathPlannerPath.fromPathFile("CoralStation-K");
            PathPlannerPath K_CoralStation = PathPlannerPath.fromPathFile("K-CoralStation");
            PathPlannerPath CoralStation_L = PathPlannerPath.fromPathFile("CoralStation-L");
            PathPlannerPath L_CoralStation = PathPlannerPath.fromPathFile("L-CoralStation");

            PathPlannerPath flippedRightReefStart_J = PathPlannerPath.fromPathFile("LeftReefStart-J").flipPath();       

            CoralReefAlignPoseAuton alignWithJ4 = new CoralReefAlignPoseAuton(ReefPosition.J, "4", false);
            CoralReefAlignPoseAuton alignWithK4 = new CoralReefAlignPoseAuton(ReefPosition.K, "4", true);
            CoralReefAlignPoseAuton alignWithL4 = new CoralReefAlignPoseAuton(ReefPosition.L, "4", false);
            
            CoralReefPoseAuton L4Pose = new CoralReefPoseAuton("4");

            return new SequentialCommandGroup(
                new InstantCommand(() -> {Robot.drivetrain.resetPose(Robot.getAlliance() == Alliance.Blue ? rightReefStart_J.getStartingHolonomicPose().get() : flippedRightReefStart_J.getStartingHolonomicPose().get()); Robot.robotMode.setCurrentMode(RobotMode.transitPose);}),
                new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(AutoBuilder.followPath(rightReefStart_J));}),
                new WaitUntilCommand(() -> Robot.robotMode.isDriveCommandFinished()).finallyDo(() -> Robot.robotMode.setCurrentMode(alignWithJ4)),
                new WaitUntilCommand(() -> RobotMode.coralFloorPose.isScheduled()).finallyDo(() -> {Robot.robotMode.setCurrentMode(RobotMode.coralStationPose); Robot.robotMode.setDriveModeCommand(AutoBuilder.followPath(J_CoralStation));}),
                new WaitUntilCommand(() -> RobotMode.transitPose.isScheduled()).finallyDo(() -> {Robot.robotMode.setDriveModeCommand(AutoBuilder.followPath(CoralStation_K));}),
                new WaitUntilCommand(() -> Robot.robotMode.isDriveCommandFinished()).finallyDo(() -> Robot.robotMode.setCurrentMode(alignWithK4)),
                new WaitUntilCommand(() -> RobotMode.coralFloorPose.isScheduled()).finallyDo(() -> {Robot.robotMode.setCurrentMode(RobotMode.coralStationPose); Robot.robotMode.setDriveModeCommand(AutoBuilder.followPath(K_CoralStation));}),
                new WaitUntilCommand(() -> RobotMode.transitPose.isScheduled()).finallyDo(() -> {Robot.robotMode.setDriveModeCommand(AutoBuilder.followPath(CoralStation_L));}),
                new WaitUntilCommand(() -> Robot.robotMode.isDriveCommandFinished()).finallyDo(() -> Robot.robotMode.setCurrentMode(alignWithL4))
            );
        } catch (Exception e) {
            return Commands.none();
        }
    }

    public static Command E4_C4_D4_CoralStation(AutoFactory factory) {

        try {
            PathPlannerPath rightReefStart_E = PathPlannerPath.fromPathFile("RightReefStart-E");
            PathPlannerPath E_CoralStation = PathPlannerPath.fromPathFile("E-CoralStation");
            PathPlannerPath CoralStation_C = PathPlannerPath.fromPathFile("CoralStation-C");
            PathPlannerPath C_CoralStation = PathPlannerPath.fromPathFile("C-CoralStation");
            PathPlannerPath CoralStation_D = PathPlannerPath.fromPathFile("CoralStation-D");
            PathPlannerPath D_CoralStation = PathPlannerPath.fromPathFile("D-CoralStation");

            PathPlannerPath flippedRightReefStart_E = PathPlannerPath.fromPathFile("RightReefStart-E").flipPath();       

            CoralReefAlignPoseAuton alignWithE4 = new CoralReefAlignPoseAuton(ReefPosition.E, "4", true);
            CoralReefAlignPoseAuton alignWithC4 = new CoralReefAlignPoseAuton(ReefPosition.C, "4", true);
            CoralReefAlignPoseAuton alignWithD4 = new CoralReefAlignPoseAuton(ReefPosition.D, "4", false);
            
            CoralReefPoseAuton L4Pose = new CoralReefPoseAuton("4");

            return new SequentialCommandGroup(
                new InstantCommand(() -> {Robot.drivetrain.resetPose(Robot.getAlliance() == Alliance.Blue ? rightReefStart_E.getStartingHolonomicPose().get() : flippedRightReefStart_E.getStartingHolonomicPose().get()); Robot.robotMode.setCurrentMode(RobotMode.transitPose);}),
                new InstantCommand(() -> {Robot.robotMode.setDriveModeCommand(AutoBuilder.followPath(rightReefStart_E));}),
                new WaitUntilCommand(() -> Robot.robotMode.isDriveCommandFinished()).finallyDo(() -> Robot.robotMode.setCurrentMode(alignWithE4)),
                new WaitUntilCommand(() -> RobotMode.coralFloorPose.isScheduled()).finallyDo(() -> {Robot.robotMode.setCurrentMode(RobotMode.coralStationPose); Robot.robotMode.setDriveModeCommand(AutoBuilder.followPath(E_CoralStation));}),
                new WaitUntilCommand(() -> RobotMode.transitPose.isScheduled()).finallyDo(() -> {Robot.robotMode.setDriveModeCommand(AutoBuilder.followPath(CoralStation_C));}),
                new WaitUntilCommand(() -> Robot.robotMode.isDriveCommandFinished()).finallyDo(() -> Robot.robotMode.setCurrentMode(alignWithC4)),
                new WaitUntilCommand(() -> RobotMode.coralFloorPose.isScheduled()).finallyDo(() -> {Robot.robotMode.setCurrentMode(RobotMode.coralStationPose); Robot.robotMode.setDriveModeCommand(AutoBuilder.followPath(C_CoralStation));}),
                new WaitUntilCommand(() -> RobotMode.transitPose.isScheduled()).finallyDo(() -> {Robot.robotMode.setDriveModeCommand(AutoBuilder.followPath(CoralStation_D));}),
                new WaitUntilCommand(() -> Robot.robotMode.isDriveCommandFinished()).finallyDo(() -> Robot.robotMode.setCurrentMode(alignWithD4))
            );
        } catch (Exception e) {
            return Commands.none();
        }
    }

    public static Command centerAutoH() {

        CoralReefAlignPoseAuton alignWithH4 = new CoralReefAlignPoseAuton(ReefPosition.H, "4", false);

        return new SequentialCommandGroup(
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(RobotMode.transitPose)),
            new WaitUntilCommand(() -> RobotMode.transit.isScheduled()),
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(alignWithH4))
        );
    }

    public static Command centerAutoG() {

        CoralReefAlignPoseAuton alignWithG4 = new CoralReefAlignPoseAuton(ReefPosition.G, "4", true);

        return new SequentialCommandGroup(
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(RobotMode.transitPose)),
            new WaitUntilCommand(() -> RobotMode.transit.isScheduled()),
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(alignWithG4))
        );
    }

    public static Command lolipopAuto(AutoFactory factory) {

        CoralReefAlignPoseAuton alignWithA = new CoralReefAlignPoseAuton(ReefPosition.A, "4", true);
        CoralReefAlignPoseAuton alignWithB = new CoralReefAlignPoseAuton(ReefPosition.B, "4", false);

        return new SequentialCommandGroup(
            new InstantCommand(() -> Robot.drivetrain.resetPose(Choreo.loadTrajectory("Wall Start To A4").get().getInitialPose(Robot.getAlliance() == Alliance.Red).get())),
            // new InstantCommand(() -> Robot.limelight.turnOnAprilTags()),
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(RobotMode.transitPose)),
            new InstantCommand(() -> Robot.robotMode.setDriveModeCommand(factory.trajectoryCmd("Wall Start To A4"))),
            new WaitUntilCommand(() -> Robot.robotMode.isDriveCommandFinished()),
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(alignWithA)),
            new WaitUntilCommand(() -> RobotMode.coralFloor.isScheduled()),
            new InstantCommand(() -> Robot.robotMode.setDriveModeCommand(factory.trajectoryCmd("A4 To Lolipop 2"))),
            new WaitUntilCommand(() -> RobotMode.transitPose.isScheduled()),
            new InstantCommand(() -> Robot.robotMode.setDriveModeCommand(factory.trajectoryCmd("Lolipop 2 To B"))),
            new WaitUntilCommand(() -> Robot.robotMode.isDriveCommandFinished()),
            new InstantCommand(() -> Robot.robotMode.setCurrentMode(alignWithB)),
            new WaitUntilCommand(() -> RobotMode.scoreCoralPose.isFinished())
        );
    }

    public static boolean isAlignCommandFinsihed() {
        return Robot.robotMode.isDriveCommandFinished();
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
