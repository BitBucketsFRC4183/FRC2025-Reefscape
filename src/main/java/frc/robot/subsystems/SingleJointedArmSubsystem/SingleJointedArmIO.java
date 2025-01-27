package frc.robot.subsystems.SingleJointedArmSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface SingleJointedArmIO {
    @AutoLog
    class ArmIOInputs {
        public double armAngle = 0.0;
        public double armAppliedVoltage = 0.0;
        public double armCurrentAmps = 0.0;
        public boolean armConnected;
        public double[] odometryTimestamps;
        public double[] odometryArmPositionsRad;
        public double encoderPosition;
    }
    public default void disable(){}
    public static void updateInputs(ArmIOInputs inputs){}

    public static void setArmMotorVoltage(double volts) {}
}
