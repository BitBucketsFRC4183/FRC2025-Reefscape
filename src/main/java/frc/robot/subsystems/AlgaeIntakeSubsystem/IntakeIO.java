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
        public double pivotVoltage = 0.0;
    }

    public default void setRunning(boolean state) {}
    public default void pivotDown() {}
    public default void pivotUp() {}
    public default boolean getIsRunning() { return false; }
    public default boolean coralInside() { return false; }
    public default void updateInputs(IntakeInputsAutoLogged inputs) {}

}
