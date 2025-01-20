package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;



public class ElevatorSetPointCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    // The subsystem the command runs on
    public ElevatorSubsystem m_elevator;
    public Timer time = new Timer();

    public ElevatorSetPointCommand(ElevatorSubsystem elevator, double setpoint) {
        m_elevator = elevator;
        addRequirements(m_elevator);
        profileGoal = new TrapezoidProfile.State(setpoint, 0);
    }
    @Override
    public void initialize(){
        time.start();
    }
    @Override
    public void execute(){

        profileSetPoint = m_elevator.elevatorProfile.calculate(time.get(), m_elevator.profileSetPoint, m_elevator.profileGoal);

        double calculatedVolts =  this.m_elevator.calculateVoltageForSetpoint(m_elevator.profileSetPoint.position);

        Logger.recordOutput("ElevatorSubsystem/target_velocity", m_elevator.profileSetPoint.velocity);
        Logger.recordOutput("ElevatorSubsystem/target_position", m_elevator.profileSetPoint.position);
        Logger.recordOutput("ElevatorSubsystem/target_voltage", calculatedVolts);

        this.m_elevator.setElevatorVoltage(calculatedVolts);

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_elevator.setElevatorVoltage(0);
        }
    }
}



