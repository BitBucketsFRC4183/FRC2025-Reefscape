package frc.robot.subsystems.ElevatorSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorEncoderIO {
    @AutoLog
    class ElevatorEncoderIOInputs {
        public double unfiliteredHeightVelocity = 0.0;
        double unfilteredLoadHeight = 0.0;
        double loadHeight = 0.0;
        double encoderPositionRads = 0;
        double encoderPositionRots = 0;
        double encoderVelocityRads = 0;
        double encoderVelocityRots = 0;
    }
    public default void updateInputs(ElevatorEncoderIOInputs inputs) {}
    public default void resetEncoderPositionWithLoadHeight() {}
}
