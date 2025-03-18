package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import org.littletonrobotics.junction.Logger;

public class ClawIntakeCommand extends Command {
    public final ClawSubsystem clawSubsystem;

    public ClawIntakeCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute() {
        this.clawSubsystem.setGrippersToVoltage(ClawConstants.grippersVoltageTarget);
    }


    public void end(boolean interrupted) {
        if (interrupted) {
            Logger.recordOutput("ClawSubsystem/gripperVolts", 0);
            this.clawSubsystem.setGrippersToVoltage(0);
        }
    }
}

