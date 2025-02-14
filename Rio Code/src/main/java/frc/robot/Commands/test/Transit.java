package frc.robot.Commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class Transit extends Command{
    
    public Transit() {
        addRequirements(Robot.robotMode);
    }

    @Override
    public void execute() {
        
        if (TriggerBoard.isNearCoralStation()) {
            Robot.robotMode.setCurrentMode(RobotMode.coralStationPose);
        } else if (TriggerBoard.isCoralButtonPressed()) {
            Robot.robotMode.setCurrentMode(RobotMode.coralFloorPose);
        }

    }

}
