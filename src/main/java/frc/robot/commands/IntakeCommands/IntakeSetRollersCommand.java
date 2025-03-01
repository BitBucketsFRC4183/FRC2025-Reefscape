package frc.robot.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakeSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class IntakeSetRollersCommand extends Command {
    public AlgaeIntakeSubsystem intake;
    boolean isIntaking = false;

    public IntakeSetRollersCommand(AlgaeIntakeSubsystem intake, boolean isIntaking) {
        this.intake = intake;
        this.isIntaking = isIntaking;
    }

    @Override
    public void execute() {
        double invertedFactor = (isIntaking && IntakeConstants.rollerInverted) ? 1 : -1;
        double calculatedVolts = IntakeConstants.rollerVoltsTarget * invertedFactor;
        Logger.recordOutput("IntakeSubsystem/targetVoltageRollers", calculatedVolts);
        this.intake.setRollersToVoltage(calculatedVolts);

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Logger.recordOutput("IntakeSubsystem/targetVoltageRollers", 0);
            this.intake.setRollersToVoltage(0);
        }
    }
}

