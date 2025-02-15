package frc.robot.Commands.StateCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMode;

public class EjectCoralPose extends Command{

    private double timer = 0;

    public EjectCoralPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        timer = Timer.getFPGATimestamp();
        Robot.manipulator.setSpeed(0.25);
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - timer) >= 1;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setCurrentMode(RobotMode.transitPose);
    }

}
