package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;

public class clawSetGrippersCommand extends Command {
    private final ClawSubsystem clawSubsystem;

    public clawSetGrippersCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    public void initialize() {
        clawSubsystem.setGrippersToVoltage();
    }

    public boolean isFinished() {
        return true;
    }
}

