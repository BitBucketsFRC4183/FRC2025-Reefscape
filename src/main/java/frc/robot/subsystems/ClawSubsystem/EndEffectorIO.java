package frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.math.controller.PIDController;

public interface EndEffectorIO {

    public final PIDController pid = new PIDController(0, 0, 0);

    public static class EndEffectorInputs {
        public double position = 0.0;
        public double velocityUnitsPerSec = 0.0;
    }

    public default void updateInputs(EndEffectorInputs inputs) {}

    public default void setVelocity(double velocity) {}

    public default void setupPID() {
        pid.setTolerance(3, 5);
        pid.setIntegratorRange(-0.5, 0.5);
    }

    public default double pidCalculate(EndEffectorEncoderIO encoder, double setpoint) {
        return pid.calculate(encoder.getDistance(), setpoint);
    }

    public default boolean atSetpoint() {
        return pid.atSetpoint();
    }


}
