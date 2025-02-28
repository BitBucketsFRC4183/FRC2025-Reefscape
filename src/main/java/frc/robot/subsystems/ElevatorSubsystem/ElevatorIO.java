package frc.robot.subsystems.ElevatorSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    class ElevatorIOInputs{
        public double elevatorAppliedVolts = 0.0;
        public double elevatorCurrentAmps = 0;
        public double elevator2AppliedVolts = 0.0;
        public double elevator2CurrentAmps = 0;
        public double elevatorMotorPositionRad = 0;
        public boolean elevatorConnected;
        public double elevatorMotorVelocityRadPerSec = 0;


    }

    public default void disable() {};
    public default void updateInputs(ElevatorIOInputs inputs) {}
    public default void setElevatorMotorVoltage(double volts) {


    }
}
