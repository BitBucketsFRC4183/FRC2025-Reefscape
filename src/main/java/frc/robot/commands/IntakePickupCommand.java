package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakeSubsystem;

public class IntakePickupCommand extends Command {
    private final AlgaeIntakeSubsystem intakeSubsystem;

    public IntakePickupCommand(AlgaeIntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    public void initialize() {
        intakeSubsystem.pickup();
    }

    public boolean isFinished() {
        return true;
    }
}

