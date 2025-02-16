package frc.robot.subsystems.GroundIntakeSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeInputsAutoLogged {
        public boolean hasCoral = false;
        public boolean isRunning = false;
    }

    public default void setRunning(boolean state) {}
    public default boolean getIsRunning() { return false; }
    public default boolean coralInside() { return false; }
    public default void updateInputs(IntakeInputsAutoLogged inputs) {}

}
