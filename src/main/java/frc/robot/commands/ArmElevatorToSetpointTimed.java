package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.ArmCommands.ArmToSetpoint;
import frc.robot.commands.ArmCommands.ArmToSetpointTimed;
import frc.robot.commands.ElevatorCommands.ElevatorSetPointCommand;
import frc.robot.commands.ElevatorCommands.ElevatorSetpointTimedCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

public class ArmElevatorToSetpointTimed extends ParallelDeadlineGroup {
    public ArmElevatorToSetpointTimed(ElevatorSubsystem elevator, ArmSubsystem arm, double elevatorSetpoint, double armSetpoint, double timeToComplete) {
        super(Commands.sequence(
                Commands.deadline(
                        Commands.waitSeconds(0.25),
                        Commands.parallel(
                                new ElevatorSetPointCommand(elevator, elevatorSetpoint))
                ),
                Commands.deadline(
                        Commands.waitSeconds(Constants.commandTimeout + 1),
                        Commands.parallel(
                                new ArmToSetpointTimed(arm, armSetpoint, timeToComplete),
                                new ElevatorSetpointTimedCommand(elevator, elevatorSetpoint, timeToComplete))
                )
        ));
    }
}
