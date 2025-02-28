package frc.robot.subsystems.SingleJointedArmSubsystem;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.constants.Constants;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.ModuleIO;
import frc.robot.subsystems.DriveSubsystem.PhoenixOdometryThread;

import java.util.Queue;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class SingleJointedArmIOTalonFX implements SingleJointedArmIO {
    private TalonFX armTalon1;
    private TalonFX armTalon2;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
            new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
            new VelocityTorqueCurrentFOC(0.0);

    public Queue<Double> timestampQueue;

    public static Queue<Double> arm1PositionQueue;

    public StatusSignal<Angle> arm1Position;

    public StatusSignal<AngularVelocity> arm1Velocity;
    public StatusSignal<Current> arm1Current;

    private final StatusSignal<Voltage> arm1AppliedVolts;


    public static Queue<Double> arm2PositionQueue;

    public StatusSignal<Angle> arm2Position;

    public StatusSignal<AngularVelocity> arm2Velocity;
    public StatusSignal<Current> arm2Current;

    private final StatusSignal<Voltage> arm2AppliedVolts;

    private final Debouncer arm1ConnectedDebounce = new Debouncer(0.5);
    private final Debouncer arm2ConnectedDebounce = new Debouncer(0.5);


    public SingleJointedArmIOTalonFX() {

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        var arm1Config = talonFXConfiguration;
        var arm2Config = talonFXConfiguration;

        arm1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        arm1Config.TorqueCurrent.PeakForwardTorqueCurrent = SingleJointedArmConstants.arm1CurrentLimit;
        arm1Config.TorqueCurrent.PeakReverseTorqueCurrent = -SingleJointedArmConstants.arm1CurrentLimit;
        arm1Config.CurrentLimits.StatorCurrentLimit = SingleJointedArmConstants.arm1CurrentLimit;
        arm1Config.CurrentLimits.StatorCurrentLimitEnable = true;

        arm2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        arm2Config.TorqueCurrent.PeakForwardTorqueCurrent = SingleJointedArmConstants.arm2CurrentLimit;
        arm2Config.TorqueCurrent.PeakReverseTorqueCurrent = -SingleJointedArmConstants.arm2CurrentLimit;
        arm2Config.CurrentLimits.StatorCurrentLimit = SingleJointedArmConstants.arm2CurrentLimit;
        arm2Config.CurrentLimits.StatorCurrentLimitEnable = true;

        armTalon1 = new TalonFX(SingleJointedArmConstants.arm1TalonID);
        armTalon2 = new TalonFX(SingleJointedArmConstants.arm2TalonID);


        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        arm1Position = armTalon1.getPosition();
        arm1PositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(armTalon1.getPosition());
        arm1Velocity = armTalon1.getVelocity();
        arm1AppliedVolts = armTalon1.getMotorVoltage();
        arm1Current = armTalon1.getStatorCurrent();

        arm2Position = armTalon2.getPosition();
        arm2PositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(armTalon2.getPosition());
        arm2Velocity = armTalon2.getVelocity();
        arm2AppliedVolts = armTalon2.getMotorVoltage();
        arm2Current = armTalon2.getStatorCurrent();


        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                arm1Position,
                arm1Velocity,
                arm1AppliedVolts,
                arm1Current,
                arm2Position,
                arm2Velocity,
                arm2AppliedVolts,
                arm2Current);
        ParentDevice.optimizeBusUtilizationForAll(armTalon1, armTalon2);

    }
    public void updateInputs(SingleJointedArmIO.ArmIOInputs inputs) {
        // Refresh all signals
        var arm1Status =
                BaseStatusSignal.refreshAll(arm1Position, arm1Velocity, arm1AppliedVolts, arm1Current);
        var arm2Status =
                BaseStatusSignal.refreshAll(arm2Position, arm2Velocity, arm2AppliedVolts, arm2Current);

        // Update drive inputs
        inputs.arm1Connected = arm1ConnectedDebounce.calculate(arm1Status.isOK());
        inputs.arm1PositionRad = Units.rotationsToRadians(arm1Position.getValueAsDouble());
        inputs.arm1VelocityRadPerSec = Units.rotationsToRadians(arm1Velocity.getValueAsDouble());
        inputs.arm1AppliedVolts = arm1AppliedVolts.getValueAsDouble();
        inputs.arm1CurrentAmps = arm1Current.getValueAsDouble();

        // Update turn inputs
        inputs.arm2Connected = arm2ConnectedDebounce.calculate(arm2Status.isOK());
        inputs.arm2PositionRad = Units.rotationsToRadians(arm2Position.getValueAsDouble());
        inputs.arm2VelocityRadPerSec = Units.rotationsToRadians(arm2Velocity.getValueAsDouble());
        inputs.arm2AppliedVolts = arm2AppliedVolts.getValueAsDouble();
        inputs.arm2CurrentAmps = arm2Current.getValueAsDouble();

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryArm1PositionsRad =
                arm1PositionQueue.stream()
                        .mapToDouble((Double value) -> Units.rotationsToRadians(value))
                        .toArray();
        inputs.odometryArm2PositionsRad =
                arm2PositionQueue.stream()
                        .mapToDouble((Double value) -> Units.rotationsToRadians(value))
                        .toArray();
        timestampQueue.clear();
        arm1PositionQueue.clear();
        arm2PositionQueue.clear();
    }
}
