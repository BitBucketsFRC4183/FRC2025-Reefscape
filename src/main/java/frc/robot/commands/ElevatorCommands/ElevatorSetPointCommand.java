package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;



public class ElevatorSetPointCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    // The subsystem the command runs on
    public ElevatorSubsystem elevator;
    public double targetHeight;

    public ElevatorSetPointCommand(ElevatorSubsystem elevator, double targetHeight) {
        this.elevator = elevator;
        this.targetHeight = targetHeight;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        elevator.elevatorPID.setGoal(targetHeight);
        Logger.recordOutput("ElevatorSubsystem/target_height", targetHeight);

    }
    @Override
    public void execute(){

        double voltsPID = elevator.elevatorPID.calculate(elevator.getLoadHeight());
        double calculatedVolts = elevator.elevatorFF.calculate(elevator.elevatorPID.getSetpoint().velocity) + voltsPID;



        Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);
        Logger.recordOutput("ElevatorSubsystem/desired_position", elevator.elevatorPID.getSetpoint().position);

        this.elevator.setElevatorVoltage(calculatedVolts);

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.setElevatorVoltage(ElevatorConstants.kG);

        }
    }
}



