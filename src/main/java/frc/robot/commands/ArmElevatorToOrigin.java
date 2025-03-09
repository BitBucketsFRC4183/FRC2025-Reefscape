package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ArmBendCommand;
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
                                Commands.waitSeconds(0.2),
                                new ArmBendCommand(arm, ArmConstants.MIN_ANGLE_RADS)
                        ,
                        Commands.parallel(
                                new ArmBendCommand(arm, ArmConstants.MIN_ANGLE_RADS),
                                new ElevatorSetPointCommand(elevator, -0.05))
                        )

                )
                )

        );
    }
}
