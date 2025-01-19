package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.ClawSubsystem.EndEffectorIO;

public class CloseClaw extends Command {
    private final ClawSubsystem clawSubsystem;

    public CloseClaw(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    public void initialize() {
        clawSubsystem.close();
    }

    public boolean isFinished() {
        return true;
    }
}
