package frc.robot.subsystems.ElevatorSubsystem;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.DriveSubsystem.SparkOdometryThread;
import org.littletonrobotics.junction.AutoLog;

import java.util.Map;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.robot.constants.DriveConstants.odometryFrequency;
import static frc.robot.constants.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;
public class ElevatorIOSparkMax implements ElevatorIO {
    private SparkMax elevatorSpark1;
    private SparkMax elevatorSpark2;
    private final RelativeEncoder elevatorMotor1Encoder;
    private final RelativeEncoder elevatorMotor2Encoder;


    private final Debouncer elevatorConnectedDebounce = new Debouncer(0.5);

    public ElevatorIOSparkMax() {
        elevatorSpark1 = new SparkMax(ElevatorConstants.elevatorSpark1CAN, SparkLowLevel.MotorType.kBrushless);
        elevatorSpark2 = new SparkMax(ElevatorConstants.elevatorSpark2CAN, SparkLowLevel.MotorType.kBrushless);

        elevatorMotor1Encoder = elevatorSpark1.getEncoder();
        elevatorMotor2Encoder = elevatorSpark2.getEncoder();


        var elevatorConfig = new SparkMaxConfig();
        elevatorConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.elevatorMotorStatorCurrentLimit, elevatorMotorSupplyCurrentLimit)
                .voltageCompensation(12.0);
        elevatorConfig
                .encoder
                .positionConversionFactor(ElevatorConstants.elevatorSparkEncoderPositionFactor)
                .velocityConversionFactor(ElevatorConstants.elevatorSparkEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        elevatorConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        var elevator1Config = elevatorConfig.inverted(elevatorSpark1Inverted);
        tryUntilOk(
                elevatorSpark1,
                5,
                () ->
                        elevatorSpark1.configure(
                                elevator1Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));

        var elevator2Config = elevatorConfig.inverted(elevatorSpark2Inverted);
        tryUntilOk(
                elevatorSpark2,
                5,
                () ->
                        elevatorSpark2.configure(
                                elevator2Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));


        tryUntilOk(elevatorSpark1, 5, () -> elevatorMotor1Encoder.setPosition(0.0));
        tryUntilOk(elevatorSpark2, 5, () -> elevatorMotor2Encoder.setPosition(0.0));


    }

    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(
                elevatorSpark1,
                new DoubleSupplier[]{elevatorSpark1::getAppliedOutput, elevatorSpark1::getBusVoltage},
                (values) -> inputs.elevatorAppliedVolts = values[0] * values[1]);

        ifOk(elevatorSpark1, elevatorSpark1::getOutputCurrent, (value) -> inputs.elevatorCurrentAmps = value);
        ifOk(
                elevatorSpark2,
                new DoubleSupplier[]{elevatorSpark2::getAppliedOutput, elevatorSpark2::getBusVoltage},
                (values) -> inputs.elevator2AppliedVolts = values[0] * values[1]);

        ifOk(elevatorSpark2, elevatorSpark2::getOutputCurrent, (value) -> inputs.elevator2CurrentAmps = value);

        inputs.elevatorMotorPositionRad = elevatorMotor1Encoder.getPosition();
        inputs.elevatorMotorVelocityRadPerSec = elevatorMotor1Encoder.getVelocity();
        inputs.elevatorConnected = elevatorConnectedDebounce.calculate(!sparkStickyFault);

    }
    public void setElevatorMotorVoltage(double volts){
        elevatorSpark1.setVoltage(volts);
        elevatorSpark2.setVoltage(volts);
    }

    @Override
    public void disable() {
        elevatorSpark1.setVoltage(0);
        elevatorSpark2.setVoltage(0);
    }
}

