package frc.robot.Commands.Intake;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class CoralFloorSpeed extends Command {

    public CoralFloorSpeed() {

        addRequirements(Robot.intake);
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeed = Robot.getDrivetrain().getChassisSpeeds();
        double metersPerSecond = Math.sqrt(((chassisSpeed.vxMetersPerSecond) * (chassisSpeed.vxMetersPerSecond)) + ((chassisSpeed.vyMetersPerSecond) * (chassisSpeed.vyMetersPerSecond)));
        Robot.intake.setWheelSpeed((Math.abs(metersPerSecond/Constants.Intake.starWheelCircumference) * 3) + 25);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
