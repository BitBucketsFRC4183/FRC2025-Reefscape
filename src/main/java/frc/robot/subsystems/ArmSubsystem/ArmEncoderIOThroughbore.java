package frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.ArmConstants;

public class ArmEncoderIOThroughbore implements ArmEncoderIO{
    private final DutyCycleEncoder armDCEncoder;
    LinearFilter armFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public ArmEncoderIOThroughbore() {
        armDCEncoder = new DutyCycleEncoder(ArmConstants.encoderChannel);
        armDCEncoder.setInverted(ArmConstants.encoderInverted);
    }

    @Override
    public void updateInputs(ArmEncoderIO.ArmEncoderIOInputs inputs) {
        inputs.encoderPositionRotsNoOffset = armDCEncoder.get();
        inputs.encoderPositionRadsNoOffset =  Units.rotationsToRadians(inputs.encoderPositionRotsNoOffset);
        inputs.encoderPositionRadsOffset =  Units.rotationsToRadians(inputs.encoderPositionRotsNoOffset) - ArmConstants.encoderOffset;

        inputs.unfilteredArmAngle = inputs.encoderPositionRadsOffset / ArmConstants.gearingRatio;
        inputs.armAngle = armFilter.calculate(inputs.unfilteredArmAngle);
    }

}
