package frc.robot.commands;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIOSparkMax;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
public class ElevatorSetPointCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    // The subsystem the command runs on
    public ElevatorSubsystem m_elevator;

    public ElevatorSetPointCommand(ElevatorSubsystem elevator, double setpoint) {
        m_elevator = elevator;
        addRequirements(m_elevator);
        m_elevator.profileGoal = new TrapezoidProfile.State(setpoint, 0);
        m_elevator.profileSetPoint = m_elevator.elevatorProfile.calculate(ElevatorConstants.kDt, elevator.profileSetPoint, elevator.profileGoal);
    }

    @Override
    public void execute(){
        this.m_elevator.moveElevatorToVelocity(1.0, m_elevator.profileSetPoint.position);
        this.m_elevator.getTotalOutput(1.0,m_elevator.profileSetPoint.position);
        Logger.recordOutput("ElevatorSubsystem/velocity", m_elevator.profileSetPoint.velocity);
        Logger.recordOutput("ElevatorSubsystem/position", m_elevator.profileSetPoint.position);
        Logger.recordOutput("ElevatorSubsystem/TotalVoltage", m_elevator.getTotalOutput(1.0,m_elevator.profileSetPoint.position));

    }

    public boolean isFinished() {
        return m_elevator.elevatorFeedback.atSetpoint();
    }
}



