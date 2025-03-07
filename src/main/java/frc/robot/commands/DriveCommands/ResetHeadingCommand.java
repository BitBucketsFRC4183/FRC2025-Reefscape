package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

public class ResetHeadingCommand extends Command {
    private DriveSubsystem driveSubsystem;

    public ResetHeadingCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute() {
        driveSubsystem.resetHeading();
    }
}
