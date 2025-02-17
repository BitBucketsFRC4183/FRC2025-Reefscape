package frc.robot.commands.ElevatorCommands;

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
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            double calculatedVolts = ElevatorConstants.kG;
            Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);
            this.elevator.setElevatorVoltage(calculatedVolts);
        }
    }
}


