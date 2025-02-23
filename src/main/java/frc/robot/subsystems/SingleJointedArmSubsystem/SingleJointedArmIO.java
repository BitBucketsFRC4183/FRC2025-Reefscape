package frc.robot.subsystems.SingleJointedArmSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface SingleJointedArmIO {

    @AutoLog
    class ArmIOInputs {
        public double armAngle = 0.0;
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
        public double[] odometryTimestamps;
        public double[] odometryArmPositionsRad;
        public double[] odometryArm1PositionsRad;
        public double[] odometryArm2PositionsRad;

        public double encoderPosition;
        public double armVelocity;


    }
    public default void disable(){}
    public default void updateInputs(ArmIOInputs inputs){}

    public default void setArmMotorVoltage(double volts) {}
}
