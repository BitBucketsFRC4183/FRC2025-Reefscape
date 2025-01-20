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

    public void goToSetpoint(double setpoint) {
        setVelocity(pidCalculate(encoder, setpoint));
        if (atSetpoint() || encoder.getStopped()) { //motor stops when setpoint is reached or claw closes on object
            setVelocity(0);
        }
    }

    public void rotateGrippers() { //rotate small wheels
        double velocity = 0.1;
        gripperWheels.set(velocity);
        try {
            Thread.sleep(ClawConstants.gripperMoveTimeMilliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        gripperWheels.set(0);
    }

    @Override
    public void setVelocity(double velocity) {
        centralWheel.set(velocity);
    }

    @Override
    public void setVoltage(double volts) {
        centralWheel.setVoltage(volts);
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.volts = 0.0; //get voltage
    }

}
