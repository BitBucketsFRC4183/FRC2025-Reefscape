package frc.robot.subsystems.SingleJointedArmSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface SingleJointedArmIO {
    @AutoLog
    class ArmIOInputs {
        public double armAngle = 0.0;
        public double armAppliedVoltage = 0.0;
        public double armCurrentAmps = 0.0;
        public boolean armConnected;
    }
    public default void setArmMotorVoltage(double volts) {
    }
}
