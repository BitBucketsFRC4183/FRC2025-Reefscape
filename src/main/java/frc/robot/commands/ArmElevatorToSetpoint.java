package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.ArmCommands.ArmBendCommand;
import frc.robot.commands.ElevatorCommands.ElevatorSetPointCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

public class ArmElevatorToSetpoint extends ParallelDeadlineGroup {
    public ArmElevatorToSetpoint(ElevatorSubsystem elevator, ArmSubsystem arm, double elevatorSetpoint, double armSetpoint) {
        super(Commands.deadline(
                Commands.waitSeconds(Constants.commandTimeout),
                Commands.parallel(
                        new ElevatorSetPointCommand(elevator, elevatorSetpoint),
                        new ArmBendCommand(arm, armSetpoint)
                )

        ));
    }
}
