package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

public class ElevatorGoToOriginTimedCommand extends Command {
    public ElevatorSubsystem elevator;
    public double timeToCompleteSeconds;
    // Constructor with parameters
    public ElevatorGoToOriginTimedCommand(ElevatorSubsystem elevator, double timeToCompleteSeconds) {
        this.elevator = elevator;
        this.timeToCompleteSeconds = timeToCompleteSeconds;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        double velocity = elevator.getLoadHeight() / timeToCompleteSeconds;
        this.elevator.elevatorPID.setConstraints(new TrapezoidProfile.Constraints(velocity, 1));
        this.elevator.elevatorPID.setGoal(new TrapezoidProfile.State(ElevatorConstants.Origin, 0));
    }

    @Override
    public void execute(){
        double calculatedVolts = elevator.elevatorFF.calculate(elevator.elevatorPID.getSetpoint().velocity) +
                elevator.elevatorPID.calculate(elevator.getLoadHeight());
        Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);
        this.elevator.setElevatorVoltage(calculatedVolts);
    }

}
