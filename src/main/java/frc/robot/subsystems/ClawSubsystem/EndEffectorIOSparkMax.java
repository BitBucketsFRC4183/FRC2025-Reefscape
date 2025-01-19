package frc.robot.subsystems.ClawSubsystem;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

//todo: make small wheels touching object rotate inwards for better hold

public class EndEffectorIOSparkMax implements EndEffectorIO {
    //private final SparkMax left;
    //private final SparkMax right;
    private final SparkMax center;
    private final EndEffectorEncoderIO encoder;

    public EndEffectorIOSparkMax(int canID, EndEffectorEncoderIOSim encoder) {
        setupPID();
        center = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless);
        this.encoder = encoder;

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
