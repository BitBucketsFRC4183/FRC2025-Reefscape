package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.ArmCommands.ArmBendCommand;
import frc.robot.commands.ElevatorCommands.ElevatorSetPointCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

public class ArmElevatorToRest extends ParallelDeadlineGroup {
    public ArmElevatorToRest(ElevatorSubsystem elevator, ArmSubsystem arm) {
        super(Commands.deadline(
                Commands.waitSeconds(Constants.commandTimeout + 1),
                Commands.sequence(
                        Commands.deadline(
                                Commands.waitSeconds(0.2),
                                new ArmBendCommand(arm, ArmConstants.MIN_ANGLE_RADS + Units.degreesToRadians(5)),
                        Commands.parallel(
                                new ArmBendCommand(arm, ArmConstants.MIN_ANGLE_RADS + Units.degreesToRadians(5)),
                                new ElevatorSetPointCommand(elevator, -0.05))

                )
                )
                )

        );
    }
}
