package frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO intake;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    public IntakeSubsystem(IntakeIO intake){
        this.intake = intake;
    }

    public void setPivotToVoltage(double voltage) {
        intake.setPivotVoltage(voltage);
    }

    public void setRollersToVoltage(double voltage) {
        intake.setRollersVoltage(voltage);
    }

    public void periodic() {
        intake.updateInputs(inputs);
        Logger.processInputs("IntakeSubsystem", (LoggableInputs) inputs);
    }

}
