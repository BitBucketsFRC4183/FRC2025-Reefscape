package frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.ArmConstants;

public class ArmEncoderIOThroughbore implements ArmEncoderIO{
    private final DutyCycleEncoder armDCEncoder;
    LinearFilter armFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public ArmEncoderIOThroughbore () {
        armDCEncoder = new DutyCycleEncoder(0, ArmConstants.fullRange, ArmConstants.expectedZero);
    }

    @Override
    public void updateInputs(ArmEncoderIO.ArmEncoderIOInputs inputs) {
        inputs.encoderPositionRots = armDCEncoder.get();
        inputs.encoderPositionRads =  Units.rotationsToRadians(inputs.encoderPositionRots);
        inputs.encoderVelocityRots = armDCEncoder.get() * armDCEncoder.getFrequency();
        inputs.encoderVelocityRads = Units.rotationsToRadians(inputs.encoderVelocityRads);

        inputs.unfilteredArmAngle = inputs.encoderPositionRads / ArmConstants.gearingRatio;
        inputs.armAngle = armFilter.calculate(inputs.unfilteredArmAngle);
    }
    @Override
    public void resetEncoderPositionWithArmAngle() {
        armDCEncoder.close();
    }
}
