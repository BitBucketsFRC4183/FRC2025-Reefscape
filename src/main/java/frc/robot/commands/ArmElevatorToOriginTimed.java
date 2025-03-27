package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.ArmCommands.ArmToSetpoint;
import frc.robot.commands.ArmCommands.ArmToSetpointTimed;
import frc.robot.commands.ElevatorCommands.ElevatorSetPointCommand;
import frc.robot.commands.ElevatorCommands.ElevatorSetpointTimedCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

public class ArmElevatorToOriginTimed extends ParallelDeadlineGroup {
    public ArmElevatorToOriginTimed(ElevatorSubsystem elevator, ArmSubsystem arm, double timeToComplete) {
        super(Commands.deadline(
                        Commands.waitSeconds(Constants.commandTimeout + 1),
                        Commands.sequence(
                                Commands.deadline(
                                        Commands.waitSeconds(0.1),
                                        new ArmToSetpoint(arm, ArmConstants.armOriginAngle)
                                )
                                ,
                                Commands.parallel(
                                        new ArmToSetpointTimed(arm, ArmConstants.armOriginAngle, timeToComplete),
                                        new ElevatorSetpointTimedCommand(elevator, ElevatorConstants.Origin, timeToComplete))
                        )

                )

        );
    }
}
