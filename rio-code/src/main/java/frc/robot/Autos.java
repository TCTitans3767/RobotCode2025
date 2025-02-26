package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Autos {

    public static AutoRoutine L1B4(AutoFactory factory) {
        final AutoRoutine routine = factory.newRoutine("L1B4");

        final AutoTrajectory StartToCoralStation = routine.trajectory("Start To Coral Station");
        final AutoTrajectory CoralStationToB4 = routine.trajectory("Coral Station To B4");

        routine.active().onTrue(new InstantCommand(
                () -> Robot.getDrivetrain().resetPose(StartToCoralStation.getInitialPose().get())
            )
        ).onTrue(StartToCoralStation.cmd());

        

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
