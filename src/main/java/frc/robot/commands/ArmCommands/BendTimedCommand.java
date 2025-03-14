package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmSubsystem;
import org.littletonrobotics.junction.Logger;

public class BendTimedCommand extends Command {

    private final SingleJointedArmSubsystem singleJointedArmSubsystem;
    public double targetAngle;
    public double timeToCompleteSeconds;

    public BendTimedCommand(SingleJointedArmSubsystem subsystem, double targetAngle, double timeToCompleteSeconds) {
        this.singleJointedArmSubsystem = subsystem;
        this.targetAngle = targetAngle;
        this.timeToCompleteSeconds = timeToCompleteSeconds;
        addRequirements(singleJointedArmSubsystem);
    }

    @Override
    public void initialize() {
        double velocity = targetAngle / timeToCompleteSeconds;
        singleJointedArmSubsystem.armFeedback.reset(singleJointedArmSubsystem.getCurrentAngle());
        singleJointedArmSubsystem.armFeedback.setGoal(new TrapezoidProfile.State(targetAngle, velocity));

        Logger.recordOutput("ArmSubsystem/target_Angle", targetAngle);
        Logger.recordOutput("ArmSubsystem/target_velocity", velocity);

    }

    @Override
    public void execute() {
        double voltsPID = singleJointedArmSubsystem.armFeedback.calculate(singleJointedArmSubsystem.getCurrentAngle());
        double calculatedVolts = singleJointedArmSubsystem.armFeedForward.calculate(singleJointedArmSubsystem.armFeedback.getSetpoint().position, singleJointedArmSubsystem.armFeedback.getSetpoint().velocity) + voltsPID;
        singleJointedArmSubsystem.hoverAngle = singleJointedArmSubsystem.getCurrentAngle();
        Logger.recordOutput("ArmSubsystem/target_voltage", calculatedVolts);
        Logger.recordOutput("ArmSubsystem/desired_position", singleJointedArmSubsystem.armFeedback.getSetpoint().position);

        this.singleJointedArmSubsystem.setArmVoltage(calculatedVolts);
    }

}
