package frc.robot.Commands.poses;

import org.ejml.dense.row.decomposition.hessenberg.TridiagonalDecompositionHouseholderOrig_FDRM;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.RobotMode;

public class CoralReefPose extends Command{
    
    public CoralReefPose() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void initialize() {
        // if (TriggerBoard.isL1Selected()) {

        // } else if (TriggerBoard.isL2Selected()) {

        // } else if (TriggerBoard.isL3Selected()) {

        // } else if (TriggerBoard.isL4Selected()) {

        // } else {
        //     Robot.joystick.setRumble(RumbleType.kBothRumble, 1);
        //     Robot.joystick.setRumble(RumbleType.kBothRumble, 0);
        //     Robot.robotMode.setCurrentMode(RobotMode.transitPose);
        // }

        Robot.manipulator.setSpeed(0);
        Robot.elevator.setPosition(1.05);
        Robot.arm.setPositon(-0.45);
    }

    @Override
    public boolean isFinished() {
        return Robot.elevator.atPosition() && Robot.arm.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.robotMode.setCurrentMode(RobotMode.coralReef);
    }

}
