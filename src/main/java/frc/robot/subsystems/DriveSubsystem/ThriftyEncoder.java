package frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class ThriftyEncoder extends AnalogEncoder {
    private final AnalogInput input;

    public ThriftyEncoder(AnalogInput input) {
        super(input);
        this.input = input;
    }

    public double getAngularVelocity() {
        return 0.0;
    }

    public double getRotation() {
        return get();
    }



}
