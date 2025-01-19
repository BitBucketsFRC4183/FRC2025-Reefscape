package frc.robot.subsystems.ClawSubsystem;
import com.revrobotics.spark.SparkMax;

//todo: make small wheels touching object rotate inwards for better hold

public class EndEffectorIOSparkMax implements EndEffectorIO {
    //private final SparkMax left;
    //private final SparkMax right;
    private final SparkMax center;
    private final EndEffectorEncoderIO encoder;

    public EndEffectorIOSparkMax(SparkMax center, EndEffectorEncoderIO encoder) {
        this.center = center;
        this.encoder = encoder;
        setupPID();
    }

    public void goToSetpoint(double setpoint) {
        setVelocity(pidCalculate(encoder, setpoint));
        if (atSetpoint()) {
            setVelocity(0);
        }
    }

    @Override
    public void setVelocity(double velocity) {
        center.set(velocity);
    }

    @Override
    public void setVoltage(double volts) {
        center.setVoltage(volts);
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.position = encoder.getDistance();
        inputs.velocityUnitsPerSec = encoder.getVelocity();
    }

}
