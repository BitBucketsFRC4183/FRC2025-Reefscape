package frc.robot.subsystems.ElevatorSubsystem;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.DriveSubsystem.SparkOdometryThread;
import org.littletonrobotics.junction.AutoLog;

import java.util.Map;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.robot.constants.DriveConstants.odometryFrequency;
import static frc.robot.constants.ElevatorConstants.pulleyRadius;
import static frc.robot.util.SparkUtil.*;
 public class ElevatorIOSparkMax implements ElevatorIO {
    private SparkBase elevatorSpark1;
    private SparkBase elevatorSpark2;
    private final RelativeEncoder elevatorEncoder;

    private final SparkClosedLoopController elevatorController;

    public static Queue<Double> timestampQueue;
    public static Queue<Double> elevatorPositionQueue;

    private final Debouncer elevatorConnectedDebounce = new Debouncer(0.5);

    public ElevatorIOSparkMax() {
        elevatorSpark1 = new SparkMax(9, SparkLowLevel.MotorType.kBrushless);
        elevatorSpark2 = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
        elevatorEncoder = elevatorSpark1.getEncoder();
        elevatorController = elevatorSpark1.getClosedLoopController();

        var elevatorConfig = new SparkFlexConfig();
        elevatorConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.elevatorSparkMotorCurrentLimit)
                .voltageCompensation(12.0);
        elevatorConfig
                .encoder
                .positionConversionFactor(ElevatorConstants.elevatorSparkEncoderPositionFactor)
                .velocityConversionFactor(ElevatorConstants.elevatorSparkEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        elevatorConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pidf(
                        ElevatorConstants.SparkP, 0.0,
                        ElevatorConstants.SparkD, 0.0);
        elevatorConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                elevatorSpark1,
                5,
                () ->
                        elevatorSpark1.configure(
                                elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));
        tryUntilOk(elevatorSpark1, 5, () -> elevatorEncoder.setPosition(0.0));
        tryUntilOk(
                elevatorSpark2,
                5,
                () ->
                        elevatorSpark2.configure(
                                elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));
        tryUntilOk(elevatorSpark2, 5, () -> elevatorEncoder.setPosition(0.0));
        timestampQueue = SparkOdometryThread.getInstance().
                makeTimestampQueue();
        elevatorPositionQueue = SparkOdometryThread.getInstance().
                registerSignal(elevatorSpark1, elevatorEncoder::getPosition);

    }

    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(
                elevatorSpark1,
                new DoubleSupplier[]{elevatorSpark1::getAppliedOutput, elevatorSpark1::getBusVoltage},
                (values) -> inputs.elevatorAppliedVolts = values[0] * values[1]);
        ifOk(elevatorSpark1, elevatorSpark1::getOutputCurrent, (value) -> inputs.elevatorCurrentAmps = new double[]{value});
        inputs.elevatorConnected = elevatorConnectedDebounce.calculate(!sparkStickyFault);
        inputs.loadHeight = (2 * Math.PI * pulleyRadius) / ((elevatorEncoder.getPosition() - inputs.lastEncoderPosition) * ElevatorConstants.gearingRatio);
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryElevatorPositionsRad =
                elevatorPositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        timestampQueue.clear();
        elevatorPositionQueue.clear();
        inputs.lastEncoderPosition = elevatorEncoder.getPosition();
    }
    public void setElevatorMotorVoltage(double volts){
        elevatorSpark1.setVoltage(volts);
        elevatorSpark2.setVoltage(volts);
    }
}

