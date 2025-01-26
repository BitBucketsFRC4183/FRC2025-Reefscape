package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

public class ElevatorGoToOriginCommand extends Command {
    public ElevatorSubsystem elevator;
    // Constructor with parameters
    public ElevatorGoToOriginCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        this.elevator.elevatorPID.setGoal(0);
    }

    @Override
    public void execute(){
        double calculatedVolts = elevator.elevatorFF.calculate(elevator.elevatorPID.getSetpoint().velocity) +
                elevator.elevatorPID.calculate(elevator.getLoadHeight());
        Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);
        this.elevator.setElevatorVoltage(calculatedVolts);
    }

}
