package frc.robot.subsystems.SingleJointedArmSubsystem;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;

public class ArmEncoderIOThroughbore implements ArmEncoderIO{
    private final DutyCycleEncoder armDCEncoder;
    LinearFilter armFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public ArmEncoderIOThroughbore () {
        armDCEncoder = new DutyCycleEncoder(0, SingleJointedArmConstants.fullRange, SingleJointedArmConstants.expectedZero);
    }

    @Override
    public void updateInputs(ArmEncoderIO.ArmEncoderIOInputs inputs) {
        inputs.encoderPositionRots = armDCEncoder.get();
        inputs.encoderPositionRads =  Units.rotationsToRadians(inputs.encoderPositionRots);
        inputs.encoderVelocityRots = armDCEncoder.get() * armDCEncoder.getFrequency();
        inputs.encoderVelocityRads = Units.rotationsToRadians(inputs.encoderVelocityRads);

        inputs.unfilteredArmAngle = inputs.encoderPositionRads / SingleJointedArmConstants.gearingRatio;
        inputs.armAngle = armFilter.calculate(inputs.unfilteredArmAngle);
    }
    @Override
    public void resetEncoderPositionWithArmAngle() {
        armDCEncoder.close();
    }
}
