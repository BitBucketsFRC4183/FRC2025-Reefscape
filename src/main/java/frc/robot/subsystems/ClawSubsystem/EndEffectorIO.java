package frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.ClawConstants;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    final PIDController pid = new PIDController(ClawConstants.kP, 0, 0);

    @AutoLog
    public class EndEffectorInputsAutoLogged {  //autolog volts
        public double centralVolts = 0.0;
        public double gripperVolts = 0.0;
    }

    public default void setupPID(double errorTolerance, double errorDerivativeTolerance, double minimumIntegral, double maximumIntegral) {
        pid.setTolerance(errorTolerance, errorDerivativeTolerance);
        pid.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    public default double pidCalculate(EndEffectorEncoderIO encoder, double setpoint) {
        return pid.calculate(encoder.getDistance(), setpoint);
    }

    public default void updateInputs(EndEffectorInputsAutoLogged inputs) {}

    public default void setCentralVelocity(double velocity) {}

    public default void setGrippersVelocity(double velocity) {}

    public default void setCentralVoltage(double volts) {}

    public default void setGrippersVoltage(double volts) {}

    public default void centralToSetpoint(double setpoint) {}

    public default void grippersToSetpoint(double setpoint) {}

}
