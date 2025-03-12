package frc.robot.Commands.AutonCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.CommandTrigger;

public class ScoreL1AutonCommand extends Command{

    double timer = 0;
    
    public ScoreL1AutonCommand() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        timer = Timer.getFPGATimestamp();
        Robot.intake.setWheelSpeed(-0.25);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - timer >= 1;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.setWheelSpeed(0);
    }

}
