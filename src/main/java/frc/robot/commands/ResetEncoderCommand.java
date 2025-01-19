package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

public class ResetEncoderCommand  extends Command implements ElevatorEncoderIO {
    public ElevatorSubsystem m_elevator;
    // Constructor with parameters
    public ResetEncoderCommand(ElevatorSubsystem elevator, ElevatorIO.ElevatorIOInputs inputs) {
        this.m_elevator = elevator;
        addRequirements(m_elevator);
        inputs.loadHeight = 0;
        Logger.recordOutput("ElevatorSubsystem/position", inputs.loadHeight);
    }

    public void execute(){

    }

}
