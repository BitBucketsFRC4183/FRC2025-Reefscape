package frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.ClawConstants;

public interface EndEffectorIO {

    final PIDController pid = new PIDController(ClawConstants.kP, 0, 0);

    public static class EndEffectorInputs {
        public double volts = 0.0;
    }

    public default void updateInputs(EndEffectorInputs inputs) {}

    public default void setCentralVelocity(double velocity) {}

    public default void setGrippersVelocity(double velocity) {}

    public default void setCentralVoltage(double volts) {}

    public default void setGrippersVoltage(double volts) {}

    public default void setupPID() {
        pid.setTolerance(3, 5);
        pid.setIntegratorRange(-0.5, 0.5);
    }

    public default double pidCalculate(EndEffectorEncoderIO encoder, double setpoint) {
        return pid.calculate(encoder.getDistance(), setpoint);
    }

    public default void centralToSetpoint(double setpoint) {}

    public default void grippersToSetpoint(double setpoint) {}

}
