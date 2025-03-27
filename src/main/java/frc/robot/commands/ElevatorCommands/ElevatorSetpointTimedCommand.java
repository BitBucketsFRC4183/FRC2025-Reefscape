package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;



public class ElevatorSetpointTimedCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    // The subsystem the command runs on
    public ElevatorSubsystem elevator;
    public double targetHeight;
    public double timeToCompleteSeconds;

    public ElevatorSetpointTimedCommand(ElevatorSubsystem elevator, double targetHeight, double timeToCompleteSeconds) {
        this.elevator = elevator;
        this.targetHeight = targetHeight;
        this.timeToCompleteSeconds = timeToCompleteSeconds;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        double velocity = (targetHeight - elevator.getLoadHeight()) / timeToCompleteSeconds;
        this.elevator.elevatorPID.setConstraints(new TrapezoidProfile.Constraints(velocity, 1));
        this.elevator.elevatorPID.setGoal(new TrapezoidProfile.State(targetHeight, 0));
        Logger.recordOutput("ElevatorSubsystem/target_height", targetHeight);
        Logger.recordOutput("ElevatorSubsystem/target_velocity", velocity);

    }
    @Override
    public void execute(){

        double voltsPID = elevator.elevatorPID.calculate(elevator.getLoadHeight());
        double calculatedVolts = elevator.elevatorFF.calculate(elevator.elevatorPID.getSetpoint().velocity) + voltsPID;

        Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);
        Logger.recordOutput("ElevatorSubsystem/desired_position", elevator.elevatorPID.getSetpoint().position);

        this.elevator.setElevatorVoltageCommandBypass(calculatedVolts);

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.setElevatorVoltage(ElevatorConstants.kG);

        }
    }
}



