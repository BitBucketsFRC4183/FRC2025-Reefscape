package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.ArmCommands.ArmToSetpoint;
import frc.robot.commands.ElevatorCommands.ElevatorSetPointCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

public class ArmElevatorToSetpoint extends ParallelDeadlineGroup {
    public ArmElevatorToSetpoint(ElevatorSubsystem elevator, ArmSubsystem arm, double elevatorSetpoint, double armSetpoint) {
        super(Commands.sequence(
                Commands.deadline(
                        Commands.waitSeconds(0.25),
                        Commands.parallel(
                                new ElevatorSetPointCommand(elevator, elevatorSetpoint))
                ),
                Commands.deadline(
                Commands.waitSeconds(Constants.commandTimeout + 1),
                Commands.parallel(
                        new ArmToSetpoint(arm, armSetpoint),
                        new ElevatorSetPointCommand(elevator, elevatorSetpoint))
                        )
                ));
    }
}
