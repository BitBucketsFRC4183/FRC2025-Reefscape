package frc.robot.subsystems.ClawSubsystem;

public interface EndEffectorEncoderIO {

    public class EncoderInputs {
        public double position = 0.0;
        public double velocityUnitsPerSec = 0.0;
    }

    public default double getVelocity() {
        return 0.0;
    }

    public default double getDistance() {
        return 0.0;
    }

    public default boolean getStopped() {
        return true;
    }

    public default void updateInputs(EncoderInputs inputs) {}

}
