package frc.robot.subsystems.AlgaeIntakeSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final IntakeIO intake;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    public AlgaeIntakeSubsystem(IntakeIO intake){
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
        Logger.processInputs("IntakeSubsystem", inputs);
    }

}
