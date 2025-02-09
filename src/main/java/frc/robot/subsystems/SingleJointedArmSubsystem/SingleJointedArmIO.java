package frc.robot.subsystems.SingleJointedArmSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface SingleJointedArmIO {

    @AutoLog
    class ArmIOInputs {
        public double armAngle = 0.0;
        public double armAppliedVoltage = 0.0;
        public double[] armCurrentAmps = new double[] {};
        public boolean armConnected;
        public double[] odometryTimestamps;
        public double[] odometryArmPositionsRad;
        public double encoderPosition;
    }
    public default void disable(){}
    public default void updateInputs(ArmIOInputs inputs){}

    public default void setArmMotorVoltage(double volts) {}
}
