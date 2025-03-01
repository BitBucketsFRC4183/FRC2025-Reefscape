package frc.robot.subsystems.AlgaeIntakeSubsystem;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.ClawSubsystem.EndEffectorEncoderIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs{
        public boolean hasCoral = false;
        public boolean isRunning = false;
        public double rollersVoltage = 0.0;
        public double pivotVoltage = 0.0;
        public double pivotVelocity;
        public double pivotPosition;
    }

    public default void setRunning(boolean state) {}
    public default void setPivotVoltage(double volts) {}
    public default void setRollersVoltage(double volts) {}
    public default boolean getIsRunning() { return false; }
    public default boolean coralInside() { return false; }
    public default void updateInputs(IntakeIOInputs inputs) {}

}
