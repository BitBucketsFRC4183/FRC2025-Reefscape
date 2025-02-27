package frc.robot.subsystems.ElevatorSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    class ElevatorIOInputs{
        public double unfilteredLoadHeight;
        public double loadHeight = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double elevatorCurrentAmps = 0;
        public double elevatorPositionRad = 0;
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryElevatorPositionsRad = new double[] {};
        public double lastEncoderPosition = 0.0;


        public boolean elevatorConnected;
        public double elevatorVelocityRadPerSec = 0;
    }

    public default void setInverted(boolean b) {};
    public default void disable() {};
    public default void stopMotor() {};

    public default void updateInputs(ElevatorIOInputs inputs) {}
    public default void setEncoderHeightValue(double height) {}
    public default void setElevatorMotorVoltage(double volts) {


    }
}
