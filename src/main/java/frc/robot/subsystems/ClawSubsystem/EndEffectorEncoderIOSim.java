package frc.robot.subsystems.ClawSubsystem;

public class EndEffectorEncoderIOSim implements EndEffectorEncoderIO {

    @Override
    public void updateInputs(EncoderInputs inputs) {
        inputs.position = getDistance();
        inputs.velocityUnitsPerSec = getVelocity();
    }
}
