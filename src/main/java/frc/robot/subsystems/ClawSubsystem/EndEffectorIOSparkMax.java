package frc.robot.subsystems.ClawSubsystem;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.IntakeConstants;

import static frc.robot.constants.DriveConstants.odometryFrequency;
import static frc.robot.util.SparkUtil.tryUntilOk;


public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final SparkMax gripperWheels; //hold object
    // private final SparkMax centralWheel; //open and close claw
    private boolean hasAlgae = false;
    private boolean hasCoral = false;
    private boolean isOpen = false;
//this should be fine? we'll see
    public EndEffectorIOSparkMax() {
        // centralWheel = new SparkMax(ClawConstants.centralID, SparkLowLevel.MotorType.kBrushless); //big
        gripperWheels = new SparkMax(ClawConstants.wheelsID, SparkLowLevel.MotorType.kBrushless); //small
        SparkMaxConfig clawConfig = new SparkMaxConfig();
        clawConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(ClawConstants.clawMotorCurrentLimit)
                .voltageCompensation(12.0);

        tryUntilOk(
                gripperWheels,
                5,
                () ->
                        gripperWheels.configure(
                                clawConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));
//        tryUntilOk(centralWheel
//                ,
//                5,
//                () ->
//                        gripperWheels.configure(
//                                clawConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));
    }

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
        //centralWheel.set(velocity);
    }

    @Override
    public void setCentralVoltage(double volts) {
        // centralWheel.setVoltage(volts);
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
        // inputs.centralVolts = centralWheel.getAppliedOutput();
        inputs.gripperVolts = gripperWheels.getAppliedOutput();
        inputs.hasCoral = getHasCoral();
        inputs.hasAlgae = getHasAlgae();
        inputs.isOpen = getIsOpen();
    }

}
