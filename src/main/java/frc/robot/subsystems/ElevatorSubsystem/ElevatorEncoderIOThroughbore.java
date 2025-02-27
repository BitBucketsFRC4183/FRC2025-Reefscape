package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ElevatorConstants;

public class ElevatorEncoderIOThroughbore implements ElevatorEncoderIO {

    private final Encoder encoder;
    private double lastTime;
    LinearFilter elevatorFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public ElevatorEncoderIOThroughbore() {
        encoder = new Encoder(ElevatorConstants.encoderA, ElevatorConstants.encoderB, ElevatorConstants.isEncoderReversed);
        encoder.setDistancePerPulse(ElevatorConstants.kEncoderDistancePerPulse);
        lastTime = Timer.getFPGATimestamp();
    }
    @Override
    public void updateInputs(ElevatorEncoderIO.ElevatorEncoderIOInputs inputs) {
        inputs.encoderPositionRots = encoder.getDistance();
        inputs.encoderVelocityRots = encoder.getDistance() / (Timer.getFPGATimestamp() - lastTime);
        lastTime = Timer.getFPGATimestamp();

        inputs.encoderVelocityRads = Units.rotationsToRadians(inputs.encoderVelocityRads);
        inputs.encoderPositionRads = Units.rotationsToRadians(inputs.encoderPositionRads);

        // 1:1 with the shaft
        inputs.unfilteredLoadHeight = inputs.encoderPositionRads * 2 * Math.PI / ElevatorConstants.pulleyRatio;
        inputs.loadHeight = elevatorFilter.calculate(inputs.unfilteredLoadHeight);
    }

    @Override
    public void resetEncoderPositionWithLoadHeight() {
        encoder.reset();
    }
}
