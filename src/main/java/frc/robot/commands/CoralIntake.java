package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ClawCommands.ClawIntakeCommand;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;



public class CoralIntake {
public CoralIntake(ElevatorSubsystem elevator, ArmSubsystem arm, ClawSubsystem claw){
    Commands.sequence(Commands.deadline(
            new ArmElevatorToOrigin(elevator, arm)),
            Commands.parallel(
                    new ClawIntakeCommand(claw)
            )
    );
}
}
