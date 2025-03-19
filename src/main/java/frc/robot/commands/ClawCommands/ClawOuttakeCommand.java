package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import org.littletonrobotics.junction.Logger;

public class ClawOuttakeCommand extends Command {
    private final ClawSubsystem clawSubsystem;


    public ClawOuttakeCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    public void execute() {
        clawSubsystem.setCentralToVoltage(ClawConstants.mainVoltageTarget);
        clawSubsystem.setGrippersToVoltage(5);
    }

    public void end(boolean interrupted) {
        if (interrupted) {
            clawSubsystem.setCentralToVoltage(0);
            clawSubsystem.setGrippersToVoltage(0);
        }
    }
}
