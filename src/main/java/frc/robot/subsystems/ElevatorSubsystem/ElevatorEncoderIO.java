package frc.robot.subsystems.ElevatorSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorEncoderIO {
    @AutoLog
    class ElevatorEncoderInputs {
        double baseHeight = 0.0;
        double voltageSupply = 0.0;
        int channelA = 0;
        int channelB = 0;
    }
    default void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {}
    default void getRotationsPerMinute() {}
}
