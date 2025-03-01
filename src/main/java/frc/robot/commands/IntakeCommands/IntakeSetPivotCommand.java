package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class IntakeSetPivotCommand extends Command {
    public IntakeSubsystem intake;
    private final double kG;
    DoubleSupplier yStickDistanceSupplier;

    public IntakeSetPivotCommand(IntakeSubsystem intake, DoubleSupplier yDoubleSupplier) {
        this.intake = intake;
        addRequirements(intake);
        this.yStickDistanceSupplier = yDoubleSupplier;
        this.kG = (Constants.currentMode != Constants.Mode.SIM) ? IntakeConstants.kG : IntakeConstants.kGSim;
    }

    @Override
    public void execute() {
        double manualVelocity = yStickDistanceSupplier.getAsDouble();
        double calculatedVolts = manualVelocity * IntakeConstants.intakeVoltageFactor;
        Logger.recordOutput("IntakeSubsystem/targetVoltagePivot", calculatedVolts);
        this.intake.setPivotToVoltage(calculatedVolts);

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            double calculatedVolts = kG;
            Logger.recordOutput("IntakeSubsystem/targetVoltagePivot", calculatedVolts);

            this.intake.setPivotToVoltage(calculatedVolts);
        }
    }
}
