package frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogEncoder;

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

    public double getAngularVelocity() {
        double deltaTime = MathSharedStore.getTimestamp() - lastTime;
        double deltaRotation = getRotation() - lastRotation;
        return deltaRotation / deltaTime;
    }

    public double getRotation() {
        return get();
    }

    public double getVoltage() {
        return input.getVoltage();
    }

    public void periodic() {
        lastTime = MathSharedStore.getTimestamp();
        lastRotation = getRotation();
    }





}
