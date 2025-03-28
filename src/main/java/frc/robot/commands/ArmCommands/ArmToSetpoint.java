package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.util.TimestampAverageBuffer;
import org.littletonrobotics.junction.Logger;

public class ArmToSetpoint extends Command {

    private final ArmSubsystem armSubsystem;
    public double targetAngle;
    private TimestampAverageBuffer timestampAverageBuffer;

    public ArmToSetpoint(ArmSubsystem subsystem, double targetAngle) {
        this.armSubsystem = subsystem;
        this.targetAngle = targetAngle;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.armFeedback.reset(armSubsystem.getCurrentAngle());
        armSubsystem.armFeedback.setGoal(targetAngle);
        // this.timestampAverageBuffer = new TimestampAverageBuffer(0.25);
        Logger.recordOutput("ArmSubsystem/target_Angle", targetAngle);

    }

    @Override
    public void execute() {
        double voltsPID = armSubsystem.armFeedback.calculate(armSubsystem.getCurrentAngle());
        double calculatedVolts = armSubsystem.armFeedForward.calculate(armSubsystem.armFeedback.getSetpoint().position, armSubsystem.armFeedback.getSetpoint().velocity) + voltsPID;
        armSubsystem.hoverAngle = armSubsystem.getCurrentAngle();
        // timestampAverageBuffer.addValue(armSubsystem.getCurrentAngle(), Timer.getFPGATimestamp());
        Logger.recordOutput("ArmSubsystem/target_voltage", calculatedVolts);
        Logger.recordOutput("ArmSubsystem/desired_position", armSubsystem.armFeedback.getSetpoint().position);
        this.armSubsystem.setArmVoltageCommandBypass(calculatedVolts);
    }

    @Override
    public boolean isFinished() {
//        if (Math.abs(timestampAverageBuffer.getAverage() - targetAngle) < Units.degreesToRadians(2)) {
//            return true;
//        } else {
//            return false;
//        }
        return armSubsystem.armFeedback.atGoal();
    };

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
//            double calculatedVolts = ArmConstants.kG * Math.cos(armSubsystem.getCurrentAngle());
//            // calculatedVolts = ArmConstants.kG;
//            Logger.recordOutput("ArmSubsystem/target_voltage", calculatedVolts);
//            this.armSubsystem.setArmVoltage(calculatedVolts);
            new ArmHoverCommand(armSubsystem).execute();

        }
    }
}
