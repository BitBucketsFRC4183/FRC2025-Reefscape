package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;

public class CloseClawCommand extends Command {
    private final ClawSubsystem clawSubsystem;

    public CloseClawCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    public void initialize() {
        clawSubsystem.close();
    }

    public boolean isFinished() {
        return true;
    }
}
