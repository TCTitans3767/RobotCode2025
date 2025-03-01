package frc.robot.Commands.AutonCommands;

import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.utils.CommandTrigger;

public class CoralStationAuton extends CommandTrigger{
    
    public CoralStationAuton(EventLoop loop) {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
        addLoop(loop);
    }

    @Override
    public void initialize() {
        Robot.arm.setPosition(0.128);
        Robot.elevator.setPosition(0.5);
        Robot.manipulator.setSpeed(-0.2);
    }

    @Override
    public boolean isFinished() {
        return TriggerBoard.isCoralInManipulator();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.manipulator.setSpeed(0);
        super.end(interrupted);
    }

}
