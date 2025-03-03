package frc.robot.subsystems.ArmSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ArmEncoderIO {
    @AutoLog
    class ArmEncoderIOInputs {
        public double encoderPositionRotsOffset = 0;
        public double armAngleDegs = 0;
        double unfilteredArmAngle = 0.0;
        double armAngle = 0.0;
        double encoderPositionRadsOffset = 0;
        double encoderPositionRadsNoOffset = 0;
        double encoderPositionRotsNoOffset = 0;
    }
    public default void updateInputs(ArmEncoderIO.ArmEncoderIOInputs inputs) {}

}
