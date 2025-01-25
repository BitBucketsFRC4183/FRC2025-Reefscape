package frc.robot.subsystems.ClawSubsystem;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.ClawConstants;


public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final SparkMax gripperWheels; //hold object
    private final SparkMax centralWheel; //open and close claw
    private final EndEffectorEncoderIO encoder;

    public EndEffectorIOSparkMax(int canID, int smallCanID, EndEffectorEncoderIOSim encoder) {
        setupPID(3.0, 5.0, -0.5, 0.5); //change pid setting
        centralWheel = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless); //big
        gripperWheels = new SparkMax(smallCanID, SparkLowLevel.MotorType.kBrushless); //small
        this.encoder = encoder;
    }

    @Override
    public void centralToSetpoint(double setpoint) { //move wheels to setpoint
        setCentralVelocity(pidCalculate(encoder, setpoint));
    }

    @Override
    public void grippersToSetpoint(double setpoint) {
        setGrippersVelocity(pidCalculate(encoder, setpoint));
    }

    @Override
    public void setGrippersVelocity(double velocity) {
        gripperWheels.set(velocity);
    }

    @Override
    public void setCentralVelocity(double velocity) {
        centralWheel.set(velocity);
    }

    @Override
    public void setCentralVoltage(double volts) {
        centralWheel.setVoltage(volts);
    }

    @Override
    public void setGrippersVoltage(double volts) {
        gripperWheels.setVoltage(volts);
    }

    @Override
    public void updateInputs(EndEffectorInputsAutoLogged inputs) {
        //inputs.centralVolts = getVoltage();
        //inputs.gripperVolts = getVoltage();
    }

}
