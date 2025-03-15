package frc.robot.subsystems.SingleJointedArmSubsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.constants.SingleJointedArmConstants;

import java.lang.Math;

public class ArmEncoderIOSim extends EncoderSim {
    private double lastTime = 0;
    private double lastAngle = SingleJointedArmConstants.MIN_ANGLE;
    private double velocityRadPerSec = 0;
    private final double circumference = 2 * SingleJointedArmConstants.armLength * Math.PI; //2 * radius * pi

    public ArmEncoderIOSim(Encoder encoder) {
        super(encoder);
    }

    public double getVelocityRadPerSec() {
        return (getAngle() - lastAngle) - (Timer.getFPGATimestamp() - lastTime);
    }

    public double getAngle() {
        return (getDistance() / circumference) * (2 * Math.PI) + SingleJointedArmConstants.MIN_ANGLE;
        //distance / circumference = percentage
        //max is 2 pi radians
        //percentage * max angle = current angle
        //add minimum angle so measurements are as if minimum is 0
    }

    public void updateMeasurements() {
        lastAngle = getAngle();
        lastTime = Timer.getFPGATimestamp();
        velocityRadPerSec = getVelocityRadPerSec();
    }

}
