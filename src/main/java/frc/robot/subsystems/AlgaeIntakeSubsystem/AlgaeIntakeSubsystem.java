package frc.robot.subsystems.AlgaeIntakeSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final IntakeIO intake;
    private final IntakeIO.IntakeInputsAutoLogged inputs = new IntakeIO.IntakeInputsAutoLogged();
    public AlgaeIntakeSubsystem(IntakeIO intake){
        this.intake = intake;
    }

    public void setPivotToVoltage() {
        intake.setPivotVoltage(IntakeConstants.pivotVoltsTarget);
    }

    public void setRollersToVoltage() {
        intake.setRollersVoltage(IntakeConstants.rollerVoltsTarget);
    }

    public void periodic() {
        intake.updateInputs(inputs);
    }

}
