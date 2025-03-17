package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;

public class ClawOuttakeCommand extends Command {
    private final ClawSubsystem clawSubsystem;


    public ClawOuttakeCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    public void initialize() {
        clawSubsystem.setCentralToVoltage(ClawConstants.mainVoltageTarget);
    }

    public boolean isFinished() {
        return true;
    }
}
