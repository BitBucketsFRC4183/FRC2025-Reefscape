package frc.robot.subsystems.ArmSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public double armAngle;
        public double armAppliedVoltage = 0.0;
        public double[] armCurrentAmps = new double[] {};
        public boolean armConnected;

        public boolean arm1Connected;
        public boolean arm2Connected;

        public double arm1PositionRad;
        public double arm2PositionRad;

        public double arm1VelocityRadPerSec;
        public double arm2VelocityRadPerSec;

        public double arm1AppliedVolts;
        public double arm2AppliedVolts;

        public double arm1CurrentAmps;
        public double arm2CurrentAmps;

        public double encoderPosition;
        public double armVelocity;
    }

    public default void disable(){}
    public default void updateInputs(ArmIOInputs inputs){}
    public default void setArmMotorVoltage(double volts) {}
}
