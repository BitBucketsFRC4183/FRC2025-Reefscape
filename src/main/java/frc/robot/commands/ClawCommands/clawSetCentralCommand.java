package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;

public class clawSetCentralCommand extends Command {
    private final ClawSubsystem clawSubsystem;

    public clawSetCentralCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    public void initialize() {
        clawSubsystem.setCentralToVoltage();
    }

    public boolean isFinished() {
        return true;
    }
}
