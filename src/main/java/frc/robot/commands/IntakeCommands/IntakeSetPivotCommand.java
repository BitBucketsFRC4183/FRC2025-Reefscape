package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakeSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class IntakeSetPivotCommand extends Command {
    public AlgaeIntakeSubsystem intake;
    private final double kG;
    DoubleSupplier yStickDistanceSupplier;

    public IntakeSetPivotCommand(AlgaeIntakeSubsystem intake, DoubleSupplier yDoubleSupplier) {
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
