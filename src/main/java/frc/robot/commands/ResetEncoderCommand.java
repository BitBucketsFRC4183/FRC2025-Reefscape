package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

public class ResetEncoderCommand  extends Command implements ElevatorEncoderIO {
    public ElevatorSubsystem elevator;
    // Constructor with parameters
    public ResetEncoderCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }
    public void intialize(){
        this.elevator.elevatorPID.setGoal(0);
    }
    public void execute(){
        double calculatedVolts = elevator.elevatorFF.calculate(elevator.elevatorPID.getSetpoint().velocity) +
                elevator.elevatorPID.calculate(elevator.getLoadHeight());
        Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);
        double loadHeight = 0.01;
        Logger.recordOutput("ElevatorSubsystem/target_height", loadHeight);
        this.elevator.setElevatorVoltage(calculatedVolts);
    }

}
