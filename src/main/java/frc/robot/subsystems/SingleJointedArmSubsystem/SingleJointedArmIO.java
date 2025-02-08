package frc.robot.subsystems.SingleJointedArmSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface SingleJointedArmIO {

    void setArmVoltage(double volts);

    void setEncoderAngleValue(double angle);

    @AutoLog
    class ArmIOInputs {
        public double unfilteredLoadAngle = 0.0;
        public double LoadAngle = 0.0;
        public double armAppliedVoltage = 0.0;
        public double armCurrentAmps = 0.0;
        public boolean armConnected;
        public double[] odometryTimestamps;
        public double[] odometryArmPositionsRad;
        public double encoderPosition;
        public double armSpeed;
    }
    public default void disable(){}
    public default void updateInputs(ArmIOInputs inputs){}

    public default void setArmMotorVoltage(double volts) {}
}
