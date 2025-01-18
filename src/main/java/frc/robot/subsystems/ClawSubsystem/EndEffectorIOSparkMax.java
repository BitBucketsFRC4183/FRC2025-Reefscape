package frc.robot.subsystems.ClawSubsystem;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final SparkMax motorController;
    private final PIDController pid;
    private final EndEffectorEncoderIO encoder;

    public EndEffectorIOSparkMax(SparkMax motorController, PIDController pid, EndEffectorEncoderIO encoder) {
        this.motorController = motorController;
        this.pid = pid;
        this.encoder = encoder;
        pid.setTolerance(3, 5);
        pid.setIntegratorRange(-0.5, 0.5);
    }

    public void open() {
        double openPoint = 1.0;
        motorController.set(pid.calculate(encoder.getDistance(), openPoint));
        if (pid.atSetpoint()) {
            motorController.set(0);
        }
    }

    public void close() {
        double closePoint = -1.0;
        motorController.set(pid.calculate(encoder.getDistance(), closePoint));
        if (pid.atSetpoint()) {
            motorController.set(0);
        }
    }

    @Override
    public void set(double velocity) {
        motorController.set(velocity);
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.position = encoder.getDistance();
        inputs.velocityUnitsPerSec = encoder.getVelocity();
    }

}
