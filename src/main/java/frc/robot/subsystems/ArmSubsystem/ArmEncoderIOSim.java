package frc.robot.subsystems.SingleJointedArmSubsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class ArmEncoderIOSim extends EncoderSim {
    private double lastTime = 0;
    private double lastDistance = 0;

    public ArmEncoderIOSim(Encoder encoder) {
        super(encoder);
    }

    public double getVelocityInSeconds() {
        return (getDistance() - lastDistance) - (Timer.getFPGATimestamp() - lastTime);
    }

    public void updateMeasurements() {
        lastDistance = getDistance();
        lastTime = Timer.getFPGATimestamp();
    }

}
