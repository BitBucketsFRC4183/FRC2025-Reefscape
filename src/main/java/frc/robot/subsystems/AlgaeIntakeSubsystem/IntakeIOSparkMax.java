package frc.robot.subsystems.AlgaeIntakeSubsystem;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {
    private final SparkMax pivot;
    private final SparkMax rollers;
    private boolean isRunning = false;
    private boolean hasCoral = false;

    public IntakeIOSparkMax(int pivotID, int rollersID) {
        this.pivot = new SparkMax(pivotID, SparkLowLevel.MotorType.kBrushless);
        this.rollers = new SparkMax(rollersID, SparkLowLevel.MotorType.kBrushless);
    }

    @Override
    public void setRunning(boolean state) {
        if (state) {
            isRunning = true;
            rollers.setVoltage(IntakeConstants.rollerVoltsTarget);
        } else {
            isRunning = false;
            rollers.setVoltage(0.0);
        }
    }

    @Override
    public void pivotDown() {
        pivot.setVoltage(IntakeConstants.pivotVoltsTarget);
        setRunning(true);
        hasCoral = true;
    }

    @Override
    public void pivotUp() {
        pivot.setVoltage(IntakeConstants.pivotVoltsTarget);
        setRunning(false);
    }

    public void setPivotVoltage(double volts) {
        pivot.setVoltage(volts);
    }

    public void setRollersVoltage(double volts) {
        rollers.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeInputsAutoLogged inputs) {
        inputs.isRunning = isRunning;
        inputs.hasCoral = hasCoral;
        inputs.rollersVoltage = rollers.getBusVoltage();
        inputs.pivotVoltage = pivot.getBusVoltage();
    }


}
