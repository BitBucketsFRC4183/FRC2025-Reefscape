package frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ArmEncoderIOThroughbore implements ArmEncoderIO{
    private final DutyCycleEncoder armDCEncoder;
    private double lastTime;
    private double lastAngle;
    LinearFilter armFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public ArmEncoderIOThroughbore() {
        armDCEncoder = new DutyCycleEncoder(ArmConstants.encoderChannel);
        armDCEncoder.setInverted(ArmConstants.encoderInverted);
        lastTime = Timer.getTimestamp();
        lastAngle = armDCEncoder.get();
    }

    @Override
    public void updateInputs(ArmEncoderIO.ArmEncoderIOInputs inputs) {
        inputs.encoderPositionRotsNoOffset = armDCEncoder.get();
        inputs.encoderPositionRotsOffset = armDCEncoder.get() - ArmConstants.encoderOffsetRots;
        inputs.encoderPositionRadsOffset = MathUtil.angleModulus(Units.rotationsToRadians(inputs.encoderPositionRotsOffset));


        inputs.unfilteredArmAngle = inputs.encoderPositionRadsOffset;
        // inputs.armAngle = armFilter.calculate(inputs.unfilteredArmAngle);
        inputs.armAngle = inputs.unfilteredArmAngle;
        inputs.armAngleDegs = Units.radiansToDegrees(inputs.armAngle);

        double dA = inputs.armAngle - lastAngle;
        double dt = Timer.getTimestamp() - lastTime;

        inputs.armVelocityRads = dA / dt;
        lastAngle = inputs.armAngle;
        lastTime = Timer.getTimestamp();
    }

}
