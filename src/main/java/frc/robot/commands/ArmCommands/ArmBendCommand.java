package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.util.TimestampAverageBuffer;
import org.dyn4j.collision.narrowphase.FallbackCondition;
import org.littletonrobotics.junction.Logger;

public class ArmBendCommand extends Command {

    private final ArmSubsystem armSubsystem;
    public double targetAngle;
    private TimestampAverageBuffer timestampAverageBuffer;

    public ArmBendCommand(ArmSubsystem subsystem, double targetAngle) {
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
        this.armSubsystem.setArmVoltage(calculatedVolts);
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
}
