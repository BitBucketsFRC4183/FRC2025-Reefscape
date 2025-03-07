package frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;

public class ThriftyEncoder extends AnalogEncoder {
    private final AnalogInput input;
    private double lastRotation;
    private double lastTime;

    public ThriftyEncoder(AnalogInput input) {
        super(input);
        this.input = input;
    }

    public ThriftyEncoder(AnalogInput input, int returnValue, int returnHalfway) {
        super(input, returnValue, returnHalfway);
        this.input = input;
    }

    public boolean getConnected() {
        return (input.getVoltage() != 0);
    }
    public double getRotationsPerSeconds() {
        double dt = Timer.getFPGATimestamp() - lastTime;
        double dR = getRotations() - lastRotation;

        lastTime = Timer.getFPGATimestamp();
        lastRotation = getRotations();
        return dR / dt;
    }

    public double getRotations() {
        return get();
    }

    public double getRadians() {
        return Units.rotationsToRadians(getRotations());
    }

    public double getPosition() {
        return getRadians();
    }

    public double getVelocity() {
        return getRadiansPerSeconds();
    }

    public double getRadiansPerSeconds() {
        return Units.rotationsToRadians(getRotationsPerSeconds());
    }

    public double getVoltage() {
        return input.getVoltage();
    }


}
