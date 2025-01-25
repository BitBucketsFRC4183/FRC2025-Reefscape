package frc.robot.subsystems.ClawSubsystem;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.ClawConstants;


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

    public void centralToSetpoint(double setpoint) {
        setCentralVelocity(pidCalculate(encoder, setpoint));
    }

    public void grippersToSetpoint(double setpoint) {
        setGrippersVelocity(pidCalculate(encoder, setpoint));
    }

    public void setGrippersVelocity(double velocity) {
        gripperWheels.set(velocity);
    }

    public void setCentralVelocity(double velocity) {
        centralWheel.set(velocity);
    }

    public void setCentralVoltage(double volts) {
        centralWheel.setVoltage(volts);
    }

    public void setGrippersVoltage(double volts) {
        gripperWheels.setVoltage(volts);
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        //inputs.volts = placeholder; //get voltage
    }

}
