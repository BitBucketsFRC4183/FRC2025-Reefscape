package frc.robot.subsystems.ClawSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorEncoderIO {

    @AutoLog
    public class EncoderInputs {
        public double position = 0.0;
        public double velocityUnitsPerSec = 0.0;
    }

    public default double getVelocity() {
        return 0.0;
    } //code encoder later

    public default double getDistance() {
        return 0.0;
    }

    public default boolean getStopped() {
        return true;
    }

    public default void updateInputs(EncoderInputs inputs) {}

}
