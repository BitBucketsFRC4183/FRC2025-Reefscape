package frc.robot.subsystems.ElevatorSubsystem;

import org.littletonrobotics.junction.AutoLog;
import com.revrobotics.RelativeEncoder;
public interface ElevatorEncoderIO {
    @AutoLog
    class ElevatorEncoderIOInputs {
        double elevatorPosition = 0.0;
        double voltageSupply = 0.0;
        int channelA = 0;
        int channelB = 0;
    }
    public default void updateInputs(ElevatorEncoderIOInputs inputs) {}
}
