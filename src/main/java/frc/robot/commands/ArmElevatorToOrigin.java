package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.ArmCommands.ArmToSetpoint;
import frc.robot.commands.ElevatorCommands.ElevatorSetPointCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

public class ArmElevatorToOrigin extends ParallelDeadlineGroup {
    public ArmElevatorToOrigin(ElevatorSubsystem elevator, ArmSubsystem arm) {
        super(Commands.deadline(
                Commands.waitSeconds(Constants.commandTimeout + 1),
                Commands.sequence(
                        Commands.deadline(
                                Commands.waitSeconds(0.3),
                                new ArmToSetpoint(arm, ArmConstants.MIN_ANGLE_RADS)
                        )
                        ,
                        Commands.parallel(
                                new ArmToSetpoint(arm, ArmConstants.MIN_ANGLE_RADS),
                                new ElevatorSetPointCommand(elevator, -0.01))
                        )

                )

        );
    }
}
