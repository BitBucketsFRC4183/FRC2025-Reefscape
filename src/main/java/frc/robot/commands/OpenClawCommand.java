package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;

public class OpenClawCommand extends Command {
    private final ClawSubsystem clawSubsystem;

    public OpenClawCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    public void initialize() {
        clawSubsystem.open();
    }

    public boolean isFinished() {
        return true;
    }
}

