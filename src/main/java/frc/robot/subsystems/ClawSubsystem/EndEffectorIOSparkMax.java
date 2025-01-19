package frc.robot.subsystems.ClawSubsystem;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import java.lang.Thread;

public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final SparkMax left;
    private final SparkMax right;
    private final SparkMax center;
    private final EndEffectorEncoderIO encoder;

    public EndEffectorIOSparkMax(int canID, int leftCanID, int rightCanID, EndEffectorEncoderIOSim encoder) {
        setupPID();
        center = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless); //big
        left = new SparkMax(leftCanID, SparkLowLevel.MotorType.kBrushless); //small
        right = new SparkMax(rightCanID, SparkLowLevel.MotorType.kBrushless); //small 2

        this.encoder = encoder;
    }

    public void goToSetpoint(double setpoint) {
        setVelocity(pidCalculate(encoder, setpoint));
        if (atSetpoint() || encoder.getStopped()) { //motor stops when setpoint is reached or claw closes on object
            setVelocity(0);
        }
    }

    public void rotateSmall() { //rotate small wheels
        double velocity = 0.1;
        left.set(velocity);
        right.set(-velocity);
        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        left.set(0);
        right.set(0);
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
