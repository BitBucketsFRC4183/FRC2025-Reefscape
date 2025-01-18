package frc.robot.subsystems.ClawSubsystem;

public interface EndEffectorIO {
    public static class EndEffectorInputs {
        public double position = 0.0;
        public double velocityUnitsPerSec = 0.0;
    }

    public default void updateInputs(EndEffectorInputs inputs) {}

    public default void set(double velocity) {}

}
