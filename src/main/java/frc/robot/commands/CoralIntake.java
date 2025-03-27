package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.ClawCommands.ClawIntakeCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;



public class CoralIntake extends ParallelDeadlineGroup {
public CoralIntake(ElevatorSubsystem elevator, ArmSubsystem arm, ClawSubsystem claw){
    super(Commands.deadline(
            Commands.waitSeconds(0.25),
            Commands.parallel(
            new ArmElevatorToSetpoint(elevator, arm, ElevatorConstants.Origin - 0.05, ArmConstants.armOriginAngle),
            new ClawIntakeCommand(claw)
            )
    ));
}
}
