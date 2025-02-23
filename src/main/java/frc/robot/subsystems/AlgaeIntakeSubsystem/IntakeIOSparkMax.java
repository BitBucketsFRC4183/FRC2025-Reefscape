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

    public IntakeIOSparkMax(int canID_left, int canID_right) {
        this.pivot = new SparkMax(canID_left, SparkLowLevel.MotorType.kBrushless);
        this.rollers = new SparkMax(canID_right, SparkLowLevel.MotorType.kBrushless);
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
    }

    @Override
    public void pivotUp() {
        pivot.setVoltage(IntakeConstants.pivotVoltsTarget);
        setRunning(false);
    }

    @Override
    public void updateInputs(IntakeInputsAutoLogged inputs) {
        inputs.isRunning = isRunning;
        inputs.hasCoral = hasCoral;
        inputs.rollersVoltage = rollers.getBusVoltage();
        inputs.pivotVoltage = pivot.getBusVoltage();
    }


}
