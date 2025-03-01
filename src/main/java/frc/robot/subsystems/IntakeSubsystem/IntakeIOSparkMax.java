package frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.IntakeConstants;
import frc.robot.util.SparkUtil;

import static frc.robot.constants.DriveConstants.odometryFrequency;

public class IntakeIOSparkMax implements IntakeIO {
    private final SparkMax pivot;
    private final SparkMax rollers;
    private final RelativeEncoder pivotEncoder;

    public IntakeIOSparkMax() {
        this.pivot = new SparkMax(IntakeConstants.pivotID, SparkLowLevel.MotorType.kBrushless);
        this.rollers = new SparkMax(IntakeConstants.rollerID, SparkLowLevel.MotorType.kBrushless);

        this.pivotEncoder = pivot.getEncoder();
        var intakeConfig = new SparkMaxConfig();
        intakeConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(IntakeConstants.intakeMotorCurrentLimit)
                .voltageCompensation(12.0);
        intakeConfig
                .encoder
                .positionConversionFactor(IntakeConstants.pivotSparkEncoderPositionFactor)
                .velocityConversionFactor(IntakeConstants.pivotSparkEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        intakeConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        SparkUtil.tryUntilOk(pivot, 5, () -> pivot.configure(
                intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));

    }

    @Override
    public void setPivotVoltage(double volts) {
        pivot.setVoltage(volts);
    }

    @Override
    public void setRollersVoltage(double volts) {
        rollers.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollersVoltage = rollers.getAppliedOutput();
        inputs.pivotVoltage = pivot.getAppliedOutput();
        inputs.pivotPosition = pivotEncoder.getPosition();
        inputs.pivotVelocity = pivotEncoder.getVelocity();
    }


}
