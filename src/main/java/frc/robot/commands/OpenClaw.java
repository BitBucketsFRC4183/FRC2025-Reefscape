package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.ClawSubsystem.EndEffectorIO;
import frc.robot.subsystems.ClawSubsystem.EndEffectorIOSparkMax;

public class OpenClaw extends Command {
    private final ClawSubsystem clawSubsystem;

    public OpenClaw(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    public void initialize() {
        clawSubsystem.open();
    }

    public boolean isFinished() {
        return true;
    }
}

