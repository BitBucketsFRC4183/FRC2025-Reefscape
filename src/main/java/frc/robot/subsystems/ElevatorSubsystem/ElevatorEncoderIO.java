package frc.robot.subsystems.ElevatorSubsystem;

import org.littletonrobotics.junction.AutoLog;
import com.revrobotics.RelativeEncoder;

public interface ElevatorEncoderIO {
    @AutoLog
    class ElevatorEncoderInputs {
        double elevatorPosition = 0.0;
        double voltageSupply = 0.0;
        int channelA = 0;
        int channelB = 0;
    }
    public static double getDistance(){}
    public default void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {}
}
