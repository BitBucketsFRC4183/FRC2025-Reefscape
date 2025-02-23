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
    private double dt;

    public ThriftyEncoder(AnalogInput input) {
        super(input);
        this.input = input;
    }

    public ThriftyEncoder(AnalogInput input, int returnValue, int returnHalfway) {
        super(input, returnValue, returnHalfway);
        this.input = input;
    }

    public double getRotationsPerSeconds() {
        double deltaRotation = getRotations() - lastRotation;
        return deltaRotation / dt;
    }

    public double getRotations() {
        return get();
    }

    public double getRadians() {
        return Units.rotationsToRadians(getRotations());
    }

    public double getRadiansPerSeconds() {
        return Units.rotationsToRadians(getRotations()) / dt;
    }

    public double getVoltage() {
        return input.getVoltage();
    }



    public void periodic() {
        lastTime = Timer.getFPGATimestamp();
        lastRotation = getRotations();
        dt = MathSharedStore.getTimestamp() - lastTime;

    }
}
