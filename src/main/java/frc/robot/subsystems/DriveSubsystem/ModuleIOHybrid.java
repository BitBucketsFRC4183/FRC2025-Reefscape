package frc.robot.subsystems.DriveSubsystem;



import static frc.robot.constants.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.constants.DriveConstants;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for TalonFX drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOHybrid implements ModuleIO {
    private final Rotation2d zeroRotation;
    private final int id;
    private final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            constants;
    // Hardware objects
    protected final TalonFX driveTalon;
    private final SparkBase turnSpark;
    private final ThriftyEncoder turnEncoder;
    // private final RelativeEncoder turnEncoder;

    // Closed loop controllers
    // private final SparkClosedLoopController turnController;
    private final ProfiledPIDController turnController;
    private final SimpleMotorFeedforward turnFF;

    // Queue inputs from odometry thread
    // private final Queue<Double> timestampSparkQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

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
        id = module;
        zeroRotation =
                switch (module) {
                    case 0 -> frontLeftZeroRotation;
                    case 1 -> frontRightZeroRotation;
                    case 2 -> backLeftZeroRotation;
                    case 3 -> backRightZeroRotation;
                    default -> new Rotation2d();
                };
        driveTalon =
                switch (module) {
                    case 0 -> new TalonFX(frontLeftDriveCanId);
                    case 1 -> new TalonFX(frontRightDriveCanId);
                    case 2 -> new TalonFX(backLeftDriveCanId);
                    case 3 -> new TalonFX(backRightDriveCanId);
                    default -> new TalonFX(0);
                };

        // Configure drive motor
        var driveConfig = new TalonFXConfiguration().withAudio(new AudioConfigs().withBeepOnBoot(true));
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = new Slot0Configs().withKP(driveKp).withKD(driveKd).withKV(driveKv).withKA(driveKa).withKS(driveKs);
        driveConfig.Feedback.SensorToMechanismRatio = driveMotorReduction;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveMotorCurrentLimit;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveMotorCurrentLimit;
        driveConfig.CurrentLimits.StatorCurrentLimit = driveMotorCurrentLimit;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));



        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
                100, drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon);

        turnSpark = new SparkMax(
                switch (module) {
                    case 0 -> frontLeftTurnCanId;
                    case 1 -> frontRightTurnCanId;
                    case 2 -> backLeftTurnCanId;
                    case 3 -> backRightTurnCanId;
                    default -> 0;
                },
                MotorType.kBrushless);




        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(turnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12.0);
//        turnConfig
//                .encoder
//                .positionConversionFactor(turnEncoderPositionFactor)
//                .velocityConversionFactor(turnEncoderVelocityFactor)
//                .uvwMeasurementPeriod(10)
//                .uvwAverageDepth(2);
//        turnConfig
//                .signals
//                .primaryEncoderPositionAlwaysOn(true)
//                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
//                .primaryEncoderVelocityAlwaysOn(true)
//                .primaryEncoderVelocityPeriodMs(20)
//                .appliedOutputPeriodMs(20)
//                .busVoltagePeriodMs(20)
//                .outputCurrentPeriodMs(20);
//        turnConfig
//                .closedLoop
//                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//                .positionWrappingEnabled(true)
//                .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
//                .pidf(turnKp, 0.0, turnKd, 0);

        SparkUtil.tryUntilOk(
                turnSpark,
                5,
                () ->
                        turnSpark.configure(
                                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // turnEncoder = turnSpark.getEncoder();
        // turnController = turnSpark.getClosedLoopController();
        
        int turnAbsoluteEncoderID =
                switch (module) {
                    case 0 -> frontLeftEncoderPort;
                    case 1 -> frontRightEncoderPort;
                    case 2 -> backLeftEncoderPort;
                    case 3 -> backRightEncoderPort;
                    default -> 0;
                };


        turnEncoder = new ThriftyEncoder(new AnalogInput(turnAbsoluteEncoderID));
        this.turnFF = new SimpleMotorFeedforward(0, DriveConstants.turnFF, 0);
        // SparkUtil.tryUntilOk(turnSpark, 5, () -> turnEncoder.setPosition(thrifty.getRadians() - zeroRotation.getRadians()));
        turnController = new ProfiledPIDController(turnKp, 0, turnKd, new TrapezoidProfile.Constraints(9999, 9999));
        turnController.enableContinuousInput(turnPIDMinInput,turnPIDMaxInput);


        // Create timestamp queue
        // In this hybrid implementation, turnEncoder positions are synced with drive encoder updates
        timestampPhoenixQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        // timestampSparkQueue = SparkOdometryThread.getInstance().makeTimestampQueue();

        drivePositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
        turnPositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(turnEncoder::getPosition);

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signal
        var driveStatus =
                BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

        turnEncoder.periodic();

        // if v = 0 lol
        // inputs.turnEncoderConnected = turnEncoder.getConnected();
        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

        // Update turn inputs
        sparkStickyFault = false;

//        inputs.turnPosition = new Rotation2d(turnEncoder.getRadians()).minus(zeroRotation);
//        currentTurnPosition = inputs.turnPosition;
//        inputs.turnVelocityRadPerSec = turnEncoder.getRadiansPerSeconds();
        ifOk(
                turnSpark,
                turnEncoder::getPosition,
                (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnSpark,
                new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getAppliedOutput},
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryPhoenixTimestamps =
                timestampPhoenixQueue.stream().mapToDouble((Double value) -> value).toArray();
//        inputs.odometrySparkTimestamps =
//                timestampSparkQueue.stream().mapToDouble((Double value) -> value).toArray();

        inputs.odometryDrivePositionsRad =
                drivePositionQueue.stream()
                        .mapToDouble(Units::rotationsToRadians)
                        .toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream()
                        .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                        .toArray(Rotation2d[]::new);


        // timestampSparkQueue.clear();
        timestampPhoenixQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
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
        double goal = rotation.getRadians();

        double turnPos =
                MathUtil.inputModulus(
                        turnEncoder.getPosition() - zeroRotation.getRadians(), turnPIDMinInput, turnPIDMaxInput);

        turnController.enableContinuousInput(turnPIDMinInput, turnPIDMaxInput);
        turnController.setGoal(goal);

        double setpointPos = turnController.getSetpoint().position;
        double output = turnController.calculate(turnPos) + turnFF.calculate(turnController.getSetpoint().velocity);

        setTurnOpenLoop(output);

        Logger.recordOutput("Module" + id + "/output", output);
        Logger.recordOutput("Module" + id + "/setpoint", setpointPos);
        Logger.recordOutput("Module" + id + "/goal", goal);
        Logger.recordOutput("Module" + id + "/position", turnPos);
//
//        turnController.setReference(setpoint, ControlType.kPosition);


    }

}