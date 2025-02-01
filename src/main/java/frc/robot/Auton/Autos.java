package frc.robot.Auton;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class Autos {
    
    public static AutoRoutine test(AutoFactory factory) {
        final AutoRoutine routine = factory.newRoutine("test");

        final AutoTrajectory trajectory = routine.trajectory("Test Path");

        routine.active().onTrue(new InstantCommand(
                () -> Robot.getDrivetrain().resetPose(trajectory.getInitialPose().get())
            )
        )
        .onTrue(trajectory.cmd());

        return routine;
    }

}
