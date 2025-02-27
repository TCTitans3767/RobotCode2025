package frc.robot.Commands.AutonCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Robot;
import frc.robot.utils.CommandTrigger;

public class ScoreL1 extends CommandTrigger{

    double timer = 0;
    
    public ScoreL1(EventLoop loop) {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
        addLoop(loop);
    }

    @Override
    public void initialize() {
        timer = Timer.getFPGATimestamp();
        Robot.intake.setWheelSpeed(-0.2);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - timer >= 1;
    }

}
