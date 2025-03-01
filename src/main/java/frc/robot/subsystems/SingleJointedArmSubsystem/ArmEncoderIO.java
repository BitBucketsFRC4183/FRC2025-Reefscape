package frc.robot.subsystems.SingleJointedArmSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ArmEncoderIO {
    @AutoLog
    class ArmEncoderIOInputs {
        double unfilteredArmAngle = 0.0;
        double armAngle = 0.0;
        double encoderPositionRads = 0;
        double encoderPositionRots = 0;
        double encoderVelocityRads = 0;
        double encoderVelocityRots = 0;
    }
    public default void updateInputs(ArmEncoderIO.ArmEncoderIOInputs inputs) {}
    public default void resetEncoderPositionWithArmAngle() {}

}
