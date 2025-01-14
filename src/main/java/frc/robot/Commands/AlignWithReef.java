package frc.robot.Commands;

import java.lang.management.BufferPoolMXBean;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTagCamera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.CommandTrigger;

public class AlignWithReef extends Command{
    
    private final AprilTagCamera camera;
    private final Drivetrain drivetrain = RobotContainer.getDrivetrain();
    private final BooleanSupplier rightBranch;

    private final PIDController xController = new PIDController(10, 0, 0);
    private final PIDController yController = new PIDController(10, 0, 0);
    private final PIDController headingController = new PIDController(7.5, 0, 0);
    
    public AlignWithReef(AprilTagCamera camera, BooleanSupplier rightBranch) {
        this.camera = camera;
        this.rightBranch = rightBranch;
    }

    @Override
    public void initialize() {
        
        

        headingController.setSetpoint(camera.getTagAngle(0));

    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
