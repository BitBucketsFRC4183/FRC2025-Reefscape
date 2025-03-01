package frc.robot.commands;

import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakeSubsystem;

public class IntakeSetPivotCommand {
    private final AlgaeIntakeSubsystem intakeSubsystem;

    public IntakeSetPivotCommand(AlgaeIntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
    }

    public void initialize() { intakeSubsystem.setPivotToVoltage(); }

    public boolean isFinished() { return true; }
}
