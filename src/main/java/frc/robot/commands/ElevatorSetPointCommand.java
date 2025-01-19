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

public class ElevatorSetPointCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    // The subsystem the command runs on
    private final ElevatorSubsystem m_elevator;

    public ElevatorSetPointCommand(ElevatorSubsystem elevator, double setpoint) {
        m_elevator = elevator;
        addRequirements(m_elevator);
        elevator.profileGoal = new TrapezoidProfile.State(setpoint, 0);
        elevator.profileSetPoint = elevator.elevatorProfile.calculate(ElevatorConstants.kDt, elevator.profileSetPoint, elevator.profileGoal);
    }

    public void execute(ElevatorSubsystem elevator){
        this.m_elevator.moveElevatorToVelocity(0.0, elevator.profileSetPoint.position);
    }

    public boolean isFinished() {
        return m_elevator.elevatorFeedback.atSetpoint();
    }
}



