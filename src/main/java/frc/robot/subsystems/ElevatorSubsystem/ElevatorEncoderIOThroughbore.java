package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ElevatorConstants;

public class ElevatorEncoderIOThroughbore implements ElevatorEncoderIO {

    private final Encoder encoder;
    private double lastTime;
    private double lastDistance;
    LinearFilter elevatorFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public ElevatorEncoderIOThroughbore() {
        encoder = new Encoder(ElevatorConstants.encoderA, ElevatorConstants.encoderB, ElevatorConstants.isEncoderReversed);
        encoder.setDistancePerPulse(ElevatorConstants.kEncoderDistancePerPulse);
        lastTime = Timer.getFPGATimestamp();
    }
    @Override
    public void updateInputs(ElevatorEncoderIO.ElevatorEncoderIOInputs inputs) {
        inputs.encoderPositionRots = encoder.getDistance();
        inputs.encoderVelocityRots = (encoder.getDistance() - lastDistance) / (Timer.getFPGATimestamp() - lastTime);
        lastTime = Timer.getFPGATimestamp();
        lastDistance = encoder.getDistance();
        inputs.encoderVelocityRads = Units.rotationsToRadians(inputs.encoderVelocityRots);
        inputs.encoderPositionRads = Units.rotationsToRadians(inputs.encoderPositionRots);

        // 1:1 with the shaft, not really lol
        inputs.unfilteredLoadHeight = inputs.encoderPositionRads / ElevatorConstants.encoderReduction;

        inputs.loadHeight = elevatorFilter.calculate(inputs.unfilteredLoadHeight);
    }

    @Override
    public void resetEncoderPositionWithLoadHeight() {
        encoder.reset();
    }
}
