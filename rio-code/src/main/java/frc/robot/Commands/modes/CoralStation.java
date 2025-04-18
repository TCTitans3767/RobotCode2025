package frc.robot.Commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.Commands.drive.AlignWithCoralStation;
import frc.robot.subsystems.RobotMode;

public class CoralStation extends Command{
    
    public CoralStation() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (TriggerBoard.isL1Selected() && Robot.manipulator.getTorque() > 40) {
            Robot.robotMode.setCurrentMode(null);
        }

        if (TriggerBoard.isCoralInManipulator()) {
            Robot.robotMode.setCurrentMode(RobotMode.coralRecievedPose);
            return;
        }

        // if (TriggerBoard.isCoralButtonPressed() && TriggerBoard.isNearCoralStation()) {
        //     Robot.robotMode.setCurrentMode(RobotMode.coralStationAlignPose);
        // }
    }

}
