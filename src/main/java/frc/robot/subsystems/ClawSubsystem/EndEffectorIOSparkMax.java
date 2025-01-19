package frc.robot.subsystems.ClawSubsystem;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import java.lang.Thread;

public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final SparkMax small;
    private final SparkMax big;
    private final EndEffectorEncoderIO encoder;

    public EndEffectorIOSparkMax(int canID, int smallCanID, EndEffectorEncoderIOSim encoder) {
        setupPID();
        big = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless); //big
        small = new SparkMax(smallCanID, SparkLowLevel.MotorType.kBrushless); //small
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
        small.set(velocity);
        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        small.set(0);
    }

    @Override
    public void setVelocity(double velocity) {
        big.set(velocity);
    }

    @Override
    public void setVoltage(double volts) {
        big.setVoltage(volts);
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.position = encoder.getDistance();
        inputs.velocityUnitsPerSec = encoder.getVelocity();
    }

}
