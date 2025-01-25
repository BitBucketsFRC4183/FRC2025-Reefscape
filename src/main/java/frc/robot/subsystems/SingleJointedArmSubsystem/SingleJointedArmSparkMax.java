package frc.robot.subsystems.SingleJointedArmSubsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.DriveSubsystem.SparkOdometryThread;

import java.util.Map;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.robot.constants.DriveConstants.odometryFrequency;
import static frc.robot.util.SparkUtil.*;

public class SingleJointedArmSparkMax implements SingleJointedArmIO{
    private final SparkBase armSpark;
    private final RelativeEncoder armEncoder;

    private final SparkClosedLoopController armController;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> elevatorPositionQueue;

    private final Debouncer armConnectedDebounce = new Debouncer(0);

    public ArmIOSparkMax(SparkBase armSpark1, SparkBase armSpark2, Queue<Double> timestampQueue, Queue<Double> armPositionQueue){
        armEncoder = armSpark1.getEncoder();
        this.armSpark1 = armSpark1;
        armController = armSpark1.getClosedLoopController();
        this.timestampQueue = timeStampQueue;
        this.armPositionQueue = armPositionQueue;

        var armConfig = new SparkFlexConfig();
        armConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(ArmConstants.armSparkMotorCurrentLimit)
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
                .pidf(  SingleJointedArmConstants.SparkkP, 0.0,
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
        tryUntil10k(
                armSpark,
                5,
                () ->
                        armSpark.configure(
                                armConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters));
        tryUntil10k(armSpark, 5, () -> armEncoder.setPosition(0.0));
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        armPositionQueue = SparkOdometryThread.getInstance().registerSignal(elevatorSpark, armEncoder::getPosition);

        public void updateInputs(SingleJointedArmIO.armIOInputs inputs){
            sparkStickyFault = false;
            if0k(
                    elevatorSpark,
                    new DoubleSupplier[]{armSpark::getAppliedOutput, armSpark1}
            )
        }
    }

}
