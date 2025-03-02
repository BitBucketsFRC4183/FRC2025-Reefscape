package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import org.littletonrobotics.junction.Logger;

public class ClawSetGrippersCommand extends Command {
    public final ClawSubsystem clawSubsystem;
    boolean isGrabbing;

    public ClawSetGrippersCommand(ClawSubsystem clawSubsystem, boolean isGrabbing) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
        this.isGrabbing = isGrabbing;
    }

    public void initialize() {
        double invertedFactor = (isGrabbing && ClawConstants.grippersInverted) ? 1 : -1;
        double calculatedVolts = ClawConstants.grippersVoltageTarget * invertedFactor;
        Logger.recordOutput("ClawSubsystem/gripperVolts", calculatedVolts);
        this.clawSubsystem.setGrippersToVoltage(calculatedVolts);
    }

    public void end(boolean interrupted) {
        if (interrupted) {
            Logger.recordOutput("ClawSubsystem/gripperVolts", 0);
            this.clawSubsystem.setGrippersToVoltage(0);
        }
    }
}

