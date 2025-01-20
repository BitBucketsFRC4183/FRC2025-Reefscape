package frc.robot.subsystems.ClawSubsystem;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.ClawConstants;

import java.lang.Thread;

public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final SparkMax gripperWheels;
    private final SparkMax centralWheel;
    private final EndEffectorEncoderIO encoder;

    public EndEffectorIOSparkMax(int canID, int smallCanID, EndEffectorEncoderIOSim encoder) {
        setupPID();
        centralWheel = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless); //big
        gripperWheels = new SparkMax(smallCanID, SparkLowLevel.MotorType.kBrushless); //small
        this.encoder = encoder;
    }

    public void goToSetpoint(SparkMax motor, double setpoint) {
        setVelocity(motor, pidCalculate(encoder, setpoint));
        if (atSetpoint() || encoder.getStopped()) { //motor stops when setpoint is reached or claw closes on object
            setVelocity(motor, 0);
        }
    }

    public void rotateGrippers() { //rotate small wheels
        double velocity = 0.1;
        setVelocity(gripperWheels, velocity);
        try {
            Thread.sleep(ClawConstants.gripperMoveTimeMilliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        setVelocity(gripperWheels, 0);
    }

    public void setVelocity(SparkMax motor, double velocity) {
        motor.set(velocity);
    }

    public void setVoltage(SparkMax motor, double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.volts = 0.0; //get voltage
    }

}
