package frc.robot.subsystems.ClawSubsystem;
import com.revrobotics.spark.SparkMax;

//todo: make small wheels touching object rotate inwards for better hold

public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final SparkMax motorController;
    private final EndEffectorEncoderIO encoder;

    public EndEffectorIOSparkMax(SparkMax motorController, EndEffectorEncoderIO encoder) {
        this.motorController = motorController;
        this.encoder = encoder;
        setupPID();
    }

    public void open() {
        double openPoint = 1.0;
        setVelocity(pidCalculate(encoder, openPoint));
        if (atSetpoint()) {
            setVelocity(0);
        }
    }

    public void close() {
        double closePoint = -1.0;
        setVelocity(pidCalculate(encoder, closePoint));
        if (encoder.getStopped()) {
            setVelocity(0);
        }
    }

    @Override
    public void setVelocity(double velocity) {
        motorController.set(velocity);
    }

    @Override
    public void setVoltage(double volts) {
        motorController.setVoltage(volts);
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.position = encoder.getDistance();
        inputs.velocityUnitsPerSec = encoder.getVelocity();
    }

}
