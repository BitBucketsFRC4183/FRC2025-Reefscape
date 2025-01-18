package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

public class ResetEncoderCommand  extends Command implements ElevatorEncoderIO {
    public ElevatorSubsystem elevator;
    // Constructor with parameters
    public ResetEncoderCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    public void execute(){

    }

}
