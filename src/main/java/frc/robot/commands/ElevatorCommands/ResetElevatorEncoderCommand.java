package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

public class ResetElevatorEncoderCommand extends Command  {
    public ElevatorSubsystem elevator;
    public ResetElevatorEncoderCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void execute(){

        elevator.resetLoadHeightEncoderValue();
    }
}
