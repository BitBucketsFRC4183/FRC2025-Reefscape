package frc.robot.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakeSubsystem;

public class IntakeSetRollersCommand extends Command {
    private final AlgaeIntakeSubsystem intakeSubsystem;

    public IntakeSetRollersCommand(AlgaeIntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    public void initialize() {
        intakeSubsystem.setRollersToVoltage();
    }

    public boolean isFinished() {
        return true;
    }
}

