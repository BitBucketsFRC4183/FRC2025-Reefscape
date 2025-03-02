package frc.robot.subsystems.ArmSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ArmEncoderIO {
    @AutoLog
    class ArmEncoderIOInputs {
        double unfilteredArmAngle = 0.0;
        double armAngle = 0.0;
        double encoderPositionRadsOffset = 0;
        double encoderPositionRadsNoOffset = 0;
        double encoderPositionRotsNoOffset = 0;
    }
    public default void updateInputs(ArmEncoderIO.ArmEncoderIOInputs inputs) {}

}
