package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandTrigger {
    private boolean isActive = false;
    private EventLoop loop;
    private Subsystem[] requirements;

    public CommandTrigger() {
        this.loop = new EventLoop();
    }

    public void addRequirements(Subsystem... requirements) {
        this.requirements = requirements;
    }

    public void addLoop(EventLoop loop) {
        this.loop = loop;
    }

    public void initialize() {
        active(true);
    }

    public void execute() {

    }

    public void end(boolean interrupted) {
        active(false);
    }

    public boolean isFinished() {
        return false;
    }

    public Command cmd() {
        return new FunctionalCommand(this::initialize, 
                                    this::execute, 
                                    this::end,
                                    this::isFinished,
                                    requirements);
    }

    public void active(boolean active) {
        isActive = false;
    }

    public Trigger done() {
        return new Trigger(
            loop,
            new BooleanSupplier() {
                boolean wasJustActive = false;

                public boolean getAsBoolean() {
                  if (isActive) {
                    wasJustActive = true;
                  } else if (wasJustActive) {
                    wasJustActive = false;
                    return true;
                  }
                  return false;
                }
            }
        );
    }

    public Trigger active() {
        return new Trigger(loop, () -> isActive);
    }

    public Trigger inactive() {
        return active().negate();
    }


}
