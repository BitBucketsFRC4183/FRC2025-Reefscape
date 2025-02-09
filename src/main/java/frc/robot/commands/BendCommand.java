package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmSubsystem;
import org.littletonrobotics.junction.Logger;

public class BendCommand extends Command {

    private final SingleJointedArmSubsystem singleJointedArmSubsystem;
    public double targetAngle;

    public BendCommand(SingleJointedArmSubsystem subsystem, double targetAngle) {
        this.singleJointedArmSubsystem = subsystem;
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        singleJointedArmSubsystem.armFeedback.setGoal(targetAngle);
        Logger.recordOutput("ArmSubsystem/target_Angle", targetAngle);

    }

    @Override
    public void execute() {
        double voltsPID = singleJointedArmSubsystem.armFeedback.calculate(singleJointedArmSubsystem.getCurrentAngle());
        double calculatedVolts = singleJointedArmSubsystem.armFeedForward.calculate(singleJointedArmSubsystem.armFeedback.getSetpoint().position, singleJointedArmSubsystem.armFeedback.getSetpoint().velocity) + voltsPID;

        Logger.recordOutput("SingleJointedArmSubsystem/target_voltage", calculatedVolts);
        Logger.recordOutput("SingleJointedArmSubsystem/desired_position", singleJointedArmSubsystem.armFeedback.getSetpoint().position);

        this.singleJointedArmSubsystem.setArmVoltage(calculatedVolts);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            singleJointedArmSubsystem.setArmVoltage(SingleJointedArmConstants.kG * Math.cos(singleJointedArmSubsystem.getCurrentAngle()));
        }
    }
}
