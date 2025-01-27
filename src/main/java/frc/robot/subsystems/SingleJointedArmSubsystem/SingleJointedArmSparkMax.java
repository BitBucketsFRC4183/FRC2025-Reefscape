package frc.robot.subsystems.SingleJointedArmSubsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.DriveSubsystem.SparkOdometryThread;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.robot.constants.DriveConstants.odometryFrequency;
import static frc.robot.util.SparkUtil.*;

public class SingleJointedArmSparkMax implements SingleJointedArmIO{
    private SparkBase armSpark;
    private RelativeEncoder armEncoder;

    private SparkClosedLoopController armController;

    private Queue<Double> timestampQueue;
    private Queue<Double> armPositionQueue;

    private final Debouncer armConnectedDebounce = new Debouncer(0);

    public void ArmIOSparkMax(SparkBase armSpark1, SparkBase armSpark2, Queue<Double> timestampQueue, Queue<Double> armPositionQueue) {
        armEncoder = armSpark.getEncoder();
        this.armSpark = armSpark;
        armController = armSpark.getClosedLoopController();
        this.timestampQueue = timestampQueue;
        this.armPositionQueue = armPositionQueue;

        var armConfig = new SparkFlexConfig();
        armConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(SingleJointedArmConstants.armSparkMotorCurrentLimit)
                .voltageCompensation(12.0);
        armConfig
                .encoder
                .positionConversionFactor(SingleJointedArmConstants.armSparkEncoderPositionFactor)
                .velocityConversionFactor(SingleJointedArmConstants.armSparkEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        armConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pidf(SingleJointedArmConstants.SparkkP, 0.0,
                        SingleJointedArmConstants.SparkkD, 0.0);
        armConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                armSpark,
                5,
                () ->
                        armSpark.configure(
                                armConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));
        tryUntilOk(armSpark, 5, () -> armEncoder.setPosition(0.0));
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        armPositionQueue = SparkOdometryThread.getInstance().registerSignal(armSpark, armEncoder::getPosition);
    }

        @Override
        public void updateInputs(SingleJointedArmIO.ArmIOInputs inputs) {
            sparkStickyFault = false;
            ifOk(
                    armSpark,
                    new DoubleSupplier[]{armSpark::getAppliedOutput, armSpark::getBusVoltage},
                    (values) -> inputs.armAppliedVoltage = values[0] * values[1]);
            ifOk(armSpark, armSpark::getOutputCurrent, (value) -> inputs.armCurrentAmps = value);
            inputs.armConnected = armConnectedDebounce.calculate(!sparkStickyFault);

            inputs.odometryTimestamps =
                    timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
            inputs.odometryArmPositionsRad =
                    armPositionQueue.stream().mapToDouble((Double value) -> value).toArray();
            timestampQueue.clear();
            armPositionQueue.clear();
            inputs.encoderPosition = armEncoder.getPosition();
        }

        @Override
        public void setArmMotorVoltage(double volts) {
            armSpark.setVoltage(volts);
        }
    }

