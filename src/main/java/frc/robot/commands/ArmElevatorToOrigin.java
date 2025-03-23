package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.ArmCommands.ArmToSetpoint;
import frc.robot.commands.ElevatorCommands.ElevatorSetPointCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

public class ArmElevatorToOrigin extends ParallelDeadlineGroup {
    public ArmElevatorToOrigin(ElevatorSubsystem elevator, ArmSubsystem arm) {
        super(Commands.deadline(
                Commands.waitSeconds(Constants.commandTimeout + 1),
                Commands.sequence(
                        Commands.deadline(
                                Commands.waitSeconds(0.1),
                                new ArmToSetpoint(arm, ArmConstants.armOriginAngle)
                        )
                        ,
                        Commands.parallel(
                                new ArmToSetpoint(arm, ArmConstants.armOriginAngle),
                                new ElevatorSetPointCommand(elevator, ElevatorConstants.Origin))
                        )

                )

        );
    }
}
