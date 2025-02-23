package frc.robot.subsystems.DriveSubsystem;



import static frc.robot.constants.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.generated.TunerConstants;
import frc.robot.util.SparkUtil;

import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOHybrid implements ModuleIO {
    private final Rotation2d zeroRotation;
    private final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            constants;
    // Hardware objects
    protected final TalonFX driveTalon;
    private final SparkBase turnSpark;
    private final CANcoder cancoder;
    private final ThriftyEncoder turnEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController turnController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue = null;

    // Timestamp inputs from Phoenix thread
    private final Queue<Double> timestampPhoenixQueue;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
            new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
            new VelocityTorqueCurrentFOC(0.0);

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    public ModuleIOHybrid(int module, SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;
        zeroRotation =
                switch (module) {
                    case 0 -> frontLeftZeroRotation;
                    case 1 -> frontRightZeroRotation;
                    case 2 -> backLeftZeroRotation;
                    case 3 -> backRightZeroRotation;
                    default -> new Rotation2d();
                };
        driveTalon = new TalonFX(constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        cancoder = new CANcoder(constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

        // Configure drive motor
        var driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted =
                constants.DriveMotorInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection =
                constants.EncoderInverted
                        ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        // Create timestamp queue
        timestampPhoenixQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        drivePositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
                DriveSubsystem.ODOMETRY_FREQUENCY, drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon);

        turnSpark = new SparkFlex(
                switch (module) {
                    case 0 -> frontLeftDriveCanId;
                    case 1 -> frontRightDriveCanId;
                    case 2 -> backLeftDriveCanId;
                    case 3 -> backRightDriveCanId;
                    default -> 0;
                },
                MotorType.kBrushless);

        int turnEncoderID =
                switch (module) {
                        case 0 -> frontLeftEncoderPort;
                        case 1 -> frontRightEncoderPort;
                        case 2 -> backLeftEncoderPort;
                        case 3 -> backRightEncoderPort;
                        default -> 0;
                };

        turnEncoder = new ThriftyEncoder(new AnalogInput(turnEncoderID));
        turnController = turnSpark.getClosedLoopController();

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(turnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12.0);
//        turnConfig
//                .absoluteEncoder
//                .inverted(turnEncoderInverted)
//                .positionConversionFactor(turnEncoderPositionFactor)
//                .velocityConversionFactor(turnEncoderVelocityFactor)
//                .averageDepth(2);
        turnConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
                .pidf(turnKp, 0.0, turnKd, 0.0);
        turnConfig
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        SparkUtil.tryUntilOk(
                turnSpark,
                5,
                () ->
                        turnSpark.configure(
                                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
//        turnPositionQueue =
//                SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);


    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signal
        var driveStatus =
                BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

        // Update turn inputs
        sparkStickyFault = false;

        inputs.turnPosition = new Rotation2d(turnEncoder.getRadians()).minus(zeroRotation);
        inputs.turnVelocityRadPerSec = turnEncoder.getRadiansPerSeconds();

        ifOk(
                turnSpark,
                new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
                drivePositionQueue.stream()
                        .mapToDouble(Units::rotationsToRadians)
                        .toArray();
//        inputs.odometryTurnPositions =
//                turnPositionQueue.stream()
//                        .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
//                        .toArray(Rotation2d[]::new);

        inputs.odometryTurnPositions = new Rotation2d[1];
        inputs.odometryTurnPositions[0] = inputs.turnPosition;

        timestampQueue.clear();
        timestampPhoenixQueue.clear();
        drivePositionQueue.clear();
        // turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> voltageRequest.withOutput(output);
                    case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
                });
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
                    case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
                });
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint =
                MathUtil.inputModulus(
                        rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
        turnController.setReference(setpoint, ControlType.kPosition);
    }
}