package frc.robot.subsystems.GroundIntakeSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeInputsAutoLogged {
        public double voltage = 0.0;
    }

    public default void setRunning(boolean state) {}
    public default boolean coralInside() { return false; }

}
