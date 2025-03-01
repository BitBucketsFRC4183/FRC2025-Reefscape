package frc.robot.subsystems.ClawSubsystem;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.ClawConstants;


public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final SparkMax gripperWheels; //hold object
    private final SparkMax centralWheel; //open and close claw
    private final EndEffectorEncoderIO encoder;
    private boolean hasAlgae = false;
    private boolean hasCoral = false;
    private boolean isOpen = false;
//this should be fine? we'll see
    public EndEffectorIOSparkMax(EndEffectorEncoderIO encoder) {
        setupPID(pid, 3.0, 5.0, -0.5, 0.5); //change pid setting
        centralWheel = new SparkMax(ClawConstants.centralID, SparkLowLevel.MotorType.kBrushless); //big
        gripperWheels = new SparkMax(ClawConstants.wheelID, SparkLowLevel.MotorType.kBrushless); //small
        this.encoder = encoder;
    }

    final PIDController pid = new PIDController(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD);


    public boolean getHasCoral() { return this.hasCoral; }

    public boolean getHasAlgae() { return this.hasAlgae; }

    public boolean getIsOpen() { return this.isOpen; }

    @Override
    public void setHasCoral(boolean setting) { this.hasCoral = setting; }

    @Override
    public void setHasAlgae(boolean setting) { this.hasAlgae = setting; }

    @Override
    public void setIsOpen(boolean setting) {this.isOpen = setting; }

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
        if (volts == ClawConstants.mainVoltageTarget) {
            setHasCoral(false);
        } else if (volts == 0) { //if closing
            setHasCoral(true);
        }
    }

    @Override
    public void setGrippersVoltage(double volts) {
        gripperWheels.setVoltage(volts);
    }

    @Override
    public void updateInputs(EndEffectorInputsAutoLogged inputs) {
        inputs.centralVolts = centralWheel.getBusVoltage();
        inputs.gripperVolts = gripperWheels.getBusVoltage();
        inputs.hasCoral = getHasCoral();
        inputs.hasAlgae = getHasAlgae();
        inputs.isOpen = getIsOpen();
    }

}
