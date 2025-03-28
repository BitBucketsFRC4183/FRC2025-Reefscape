package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.util.TimestampAverageBuffer;
import org.littletonrobotics.junction.Logger;

import java.sql.Time;
import java.sql.Timestamp;


public class ElevatorSetPointCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    // The subsystem the command runs on
    public ElevatorSubsystem elevator;
    public double targetHeight;
    // private TimestampAverageBuffer timestampAverageBuffer;

    public ElevatorSetPointCommand(ElevatorSubsystem elevator, double targetHeight) {
        this.elevator = elevator;
        this.targetHeight = targetHeight;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        elevator.elevatorPID.setGoal(targetHeight);
        elevator.elevatorPID.reset(elevator.getLoadHeight());
        // this.timestampAverageBuffer = new TimestampAverageBuffer(0.25);
        Logger.recordOutput("ElevatorSubsystem/target_height", targetHeight);

    }
    @Override
    public void execute(){

        double voltsPID = elevator.elevatorPID.calculate(elevator.getLoadHeight());
        // double calculatedVolts = elevator.elevatorFF.calculateWithVelocities(elevator.getElevatorHeightSpeed(),  elevator.elevatorPID.getSetpoint().velocity) + voltsPID;
        // timestampAverageBuffer.addValue(elevator.getLoadHeight(), Timer.getFPGATimestamp());
        double calculatedVolts = elevator.elevatorFF.calculate(elevator.elevatorPID.getSetpoint().velocity) + voltsPID;

        Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);
        Logger.recordOutput("ElevatorSubsystem/desired_position", elevator.elevatorPID.getSetpoint().position);
        Logger.recordOutput("ElevatorSubsystem/desired_velocity", elevator.elevatorPID.getSetpoint().velocity);
        Logger.recordOutput("ElevatorSubsystem/current_velocity", elevator.getElevatorHeightSpeed());


        this.elevator.setElevatorVoltageCommandBypass(calculatedVolts);

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.setElevatorVoltage(ElevatorConstants.kG);
        }
    }

    @Override
    public boolean isFinished() {
//        if (Math.abs(timestampAverageBuffer.getAverage() - targetHeight) < 0.02) {
//            return true;
//        } else {
//            return false;
//        }
        return elevator.elevatorPID.atGoal();
    }
}



