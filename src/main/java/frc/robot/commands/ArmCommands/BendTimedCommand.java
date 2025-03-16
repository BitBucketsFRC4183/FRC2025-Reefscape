package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import org.littletonrobotics.junction.Logger;

public class BendTimedCommand extends Command {

    private final ArmSubsystem armSubsystem;
    public double targetAngle;
    public double timeToCompleteSeconds;

    public BendTimedCommand(ArmSubsystem subsystem, double targetAngle, double timeToCompleteSeconds) {
        this.armSubsystem = subsystem;
        this.targetAngle = targetAngle;
        this.timeToCompleteSeconds = timeToCompleteSeconds;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        double velocity = targetAngle / timeToCompleteSeconds;
        armSubsystem.armFeedback.reset(armSubsystem.getCurrentAngle());
        this.armSubsystem.armFeedback.setConstraints(new TrapezoidProfile.Constraints(velocity, 1));
        armSubsystem.armFeedback.setGoal(new TrapezoidProfile.State(targetAngle, 0));

        Logger.recordOutput("ArmSubsystem/target_Angle", targetAngle);
        Logger.recordOutput("ArmSubsystem/target_velocity", velocity);

    }

    @Override
    public void execute() {
        double voltsPID = armSubsystem.armFeedback.calculate(armSubsystem.getCurrentAngle());
        double calculatedVolts = armSubsystem.armFeedForward.calculate(armSubsystem.armFeedback.getSetpoint().position, armSubsystem.armFeedback.getSetpoint().velocity) + voltsPID;
        armSubsystem.hoverAngle = armSubsystem.getCurrentAngle();
        Logger.recordOutput("ArmSubsystem/target_voltage", calculatedVolts);
        Logger.recordOutput("ArmSubsystem/desired_position", armSubsystem.armFeedback.getSetpoint().position);

        this.armSubsystem.setArmVoltage(calculatedVolts);
    }

}
