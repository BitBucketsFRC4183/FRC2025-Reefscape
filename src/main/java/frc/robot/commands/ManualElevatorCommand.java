package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ManualElevatorCommand extends Command {
    private static final double DEADBAND = 0.1;
    public ElevatorSubsystem elevator;
    DoubleSupplier yStickDistanceSupplier;

    public ManualElevatorCommand(ElevatorSubsystem elevator, DoubleSupplier yDoubleSupplier) {
        this.elevator = elevator;
        addRequirements(elevator);
        this.yStickDistanceSupplier = yDoubleSupplier;
    }

    @Override
    public void execute() {
        double manualVelocity = yStickDistanceSupplier.getAsDouble() * -1.0;
        double calculatedVolts = elevator.elevatorFF.calculate(manualVelocity);
        Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);
        this.elevator.setElevatorVoltage(calculatedVolts);
        if (elevator.getLoadHeight() >= ElevatorConstants.maxHeight) {
            // We are going up and top limit is tripped so stop
            elevator.setElevatorVoltage(ElevatorConstants.kGSim);
        }
        if (elevator.getLoadHeight() <= ElevatorConstants.minHeight) {
            // We are going down and bottom limit is tripped so stop
            elevator.setElevatorVoltage(ElevatorConstants.kGSim);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            double calculatedVolts = ElevatorConstants.kGSim;
            Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);
            this.elevator.setElevatorVoltage(calculatedVolts);
        }
    }
}


