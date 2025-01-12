package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs{
        public double loadHeight = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps = new double[] {};
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryElevatorPositionsRad = new double[] {};


    }

    public default void setInverted(boolean b) {};
    public default void disable() {};
    public default void stopMotor() {};

    public default void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {}

    public default void setBothElevatorMotorVoltages(double volts) {
    }
}
