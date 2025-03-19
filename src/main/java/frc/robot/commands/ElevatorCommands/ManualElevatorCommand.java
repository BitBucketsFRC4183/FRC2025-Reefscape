package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ManualElevatorCommand extends Command {
    public ElevatorSubsystem elevator;
    private final double kG;
    DoubleSupplier yStickDistanceSupplier;

    public ManualElevatorCommand(ElevatorSubsystem elevator, DoubleSupplier yDoubleSupplier) {
        this.elevator = elevator;
        addRequirements(elevator);
        this.yStickDistanceSupplier = yDoubleSupplier;
        this.kG = (Constants.currentMode != Constants.Mode.SIM) ? ElevatorConstants.kG : ElevatorConstants.kGSim;
    }

    @Override
    public void execute() {
        double manualVelocity = yStickDistanceSupplier.getAsDouble() * 1;
        double calculatedVolts = elevator.elevatorFF.calculate(manualVelocity);
        Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);
        this.elevator.setElevatorVoltage(calculatedVolts);
//        if (elevator.getLoadHeight() >= ElevatorConstants.maxHeight) {
//            // We are going up and top limit is tripped so stop
//            elevator.setElevatorVoltage(kG);
//        }
//        if (elevator.getLoadHeight() <= ElevatorConstants.minHeight) {
//            // We are going down and bottom limit is tripped so stop
//            elevator.setElevatorVoltage(kG);
//        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            double calculatedVolts = kG;
            Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);
            this.elevator.setElevatorVoltage(calculatedVolts);
        }
    }
}


