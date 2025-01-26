package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

public class ResetElevatorEncoderCommand extends Command implements ElevatorEncoderIO {
    public ElevatorSubsystem elevator;
    double loadHeight = 0;

    public ResetElevatorEncoderCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void execute(){
        this.loadHeight = 0.01;
        Logger.recordOutput("ElevatorSubsystem/loadHeight", loadHeight);

    }
}
