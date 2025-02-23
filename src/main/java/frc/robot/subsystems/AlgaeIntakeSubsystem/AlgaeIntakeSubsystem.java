package frc.robot.subsystems.AlgaeIntakeSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final IntakeIO intake;
    private final IntakeIO.IntakeInputsAutoLogged inputs = new IntakeIO.IntakeInputsAutoLogged();
    public AlgaeIntakeSubsystem(IntakeIO intake){
        this.intake = intake;
    }

    public void pickup() {
        intake.pivotDown();
        intake.pivotUp();
    }

    public void periodic() {
        intake.updateInputs(inputs);
    }


}
