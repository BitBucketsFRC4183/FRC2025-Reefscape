package frc.robot.subsystems.AlgaeIntakeSubsystem;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.ClawSubsystem.EndEffectorEncoderIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeInputsAutoLogged {
        public boolean hasCoral = false;
        public boolean isRunning = false;
        public double rollersVoltage = 0.0;
    }

    public default void setupPID(PIDController pid, double errorTolerance, double errorDerivativeTolerance, double minimumIntegral, double maximumIntegral) {
        pid.setTolerance(errorTolerance, errorDerivativeTolerance);
        pid.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    public default double pidCalculate(PIDController pid, IntakeEncoderIO encoder, double setpoint) {
        return pid.calculate(encoder.getPivotDistance(), setpoint);
    }
    public default void setRunning(boolean state) {}
    public default void pivotDown() {}
    public default void pivotUp() {}
    public default boolean getIsRunning() { return false; }
    public default boolean coralInside() { return false; }
    public default void updateInputs(IntakeInputsAutoLogged inputs) {}

}
