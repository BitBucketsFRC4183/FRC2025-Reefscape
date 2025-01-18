package frc.robot.subsystems.ClawSubsystem;

public interface EndEffectorEncoderIO {

    public default double getVelocity() {
        return 0.0;
    }

    public default double getDistance() {
        return 0.0;
    }

}
