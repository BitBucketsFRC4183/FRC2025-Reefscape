package frc.robot.subsystems.ElevatorSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    ElevatorIO.ElevatorIOInputs ElevatorIOInputs = new ElevatorIOInputs(); ;

    @AutoLog
    class ElevatorIOInputs{
        public double unfilteredLoadHeight;
        public double loadHeight = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps = new double[] {};
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryElevatorPositionsRad = new double[] {};
        public double lastEncoderPosition = 0.0;


        public boolean elevatorConnected;
        public double elevatorSpeed;
    }

    public default void setInverted(boolean b) {};
    public default void disable() {};
    public default void stopMotor() {};

    public default void updateInputs(ElevatorIOInputs inputs) {}
    public default void setEncoderHeightValue(double height) {}
    public default void setElevatorMotorVoltage(double volts) {


    }
}
