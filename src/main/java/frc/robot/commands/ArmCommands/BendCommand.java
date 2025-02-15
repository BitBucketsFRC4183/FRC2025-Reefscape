package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmSubsystem;
import org.littletonrobotics.junction.Logger;

public class BendCommand extends Command {

    private final SingleJointedArmSubsystem singleJointedArmSubsystem;
    public double targetAngle;

    public BendCommand(SingleJointedArmSubsystem subsystem, double targetAngle) {
        this.singleJointedArmSubsystem = subsystem;
        this.targetAngle = targetAngle;
        addRequirements(singleJointedArmSubsystem);
    }

    @Override
    public void initialize() {
        singleJointedArmSubsystem.armFeedback.setGoal(targetAngle);
        Logger.recordOutput("ArmSubsystem/target_Angle", targetAngle);

    }

    @Override
    public void execute() {
        double voltsPID = singleJointedArmSubsystem.armFeedback.calculate(singleJointedArmSubsystem.getCurrentAngle());
        double calculatedVolts = singleJointedArmSubsystem.armFeedForward.calculateWithVelocities(singleJointedArmSubsystem.getCurrentAngle(), singleJointedArmSubsystem.armFeedback.getSetpoint().velocity, singleJointedArmSubsystem.armFeedback.getSetpoint().velocity) + voltsPID;
        singleJointedArmSubsystem.hoverAngle = singleJointedArmSubsystem.getCurrentAngle();
        Logger.recordOutput("ArmSubsystem/current_setpoint", singleJointedArmSubsystem.armFeedback.getSetpoint().position);
        Logger.recordOutput("ArmSubsystem/target_voltage", calculatedVolts);
        Logger.recordOutput("ArmSubsystem/desired_position", singleJointedArmSubsystem.armFeedback.getSetpoint().position);

        this.singleJointedArmSubsystem.setArmVoltage(calculatedVolts);
    }

}
