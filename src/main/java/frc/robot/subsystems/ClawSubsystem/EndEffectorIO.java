package frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.ClawConstants;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    @AutoLog
    public class EndEffectorInputsAutoLogged {  //autolog volts
        public double centralVolts = 0.0;
        public double gripperVolts = 0.0;
        public boolean hasCoral = false;
        public boolean hasAlgae = false;
        public boolean isOpen = false;
    }

    public default void setupPID(PIDController pid, double errorTolerance, double errorDerivativeTolerance, double minimumIntegral, double maximumIntegral) {
        pid.setTolerance(errorTolerance, errorDerivativeTolerance);
        pid.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    public default void setHasAlgae(boolean setting) {}

    public default void setHasCoral(boolean setting) {}

    public default void setIsOpen(boolean setting) {}

    public default void updateInputs(EndEffectorInputsAutoLogged inputs) {}

    public default void open() {}

    public default void close() {}

    public default void setCentralVelocity(double velocity) {}

    public default void setGrippersVelocity(double velocity) {}

    public default void setCentralVoltage(double volts) {}

    public default void setGrippersVoltage(double volts) {}

}
