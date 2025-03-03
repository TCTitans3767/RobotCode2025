package frc.robot.Commands.AutonCommands;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.utils.CommandTrigger;

public class CoralStationAutonCommand extends Command{
    
    public CoralStationAutonCommand() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
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
    }

}
