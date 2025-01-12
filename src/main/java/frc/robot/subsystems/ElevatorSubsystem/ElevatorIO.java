package frc.robot.subsystems.ElevatorSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs{
        public double loadHeight = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps = new double[] {};
    }

    public default void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {}

    public default void setBothElevatorMotorVoltages(double volts) {}
}
