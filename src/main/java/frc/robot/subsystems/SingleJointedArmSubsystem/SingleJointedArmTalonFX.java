package frc.robot.subsystems.SingleJointedArmSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.DriveSubsystem.ModuleIO;
import frc.robot.subsystems.DriveSubsystem.PhoenixOdometryThread;
import frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmIO;

import java.util.Queue;

public class SingleJointedArmTalonFX implements SingleJointedArmIO{

    private final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration>

    // Hardware objects
    private final TalonFX armTalon1;
    private final TalonFX armTalon2;

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

    // Timestamp inputs from Phoenix thread
    private final Queue<Double> timestampQueue;


    private final StatusSignal<Angle> arm1Position;
    private final Queue<Double> arm1PositionQueue;
    private final StatusSignal<AngularVelocity> arm1Velocity;
    private final StatusSignal<Voltage> arm1AppliedVolts;
    private final StatusSignal<Current> arm1Current;

    // Inputs from turn motor
    private final StatusSignal<Angle> arm2AbsolutePosition;
    private final StatusSignal<Angle> arm2Position;
    private final Queue<Double> arm2PositionQueue;
    private final StatusSignal<AngularVelocity> arm2Velocity;
    private final StatusSignal<Voltage> arm2AppliedVolts;
    private final StatusSignal<Current> arm2Current;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    public SingleJointedArmTalonFX() {

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        var singleJointedArmConfig = talonFXConfiguration;

        singleJointedArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        singleJointedArmConfig.TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.elevatorMotorCurrentLimit;
        singleJointedArmConfig.TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.elevatorMotorCurrentLimit;
        singleJointedArmConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.elevatorMotorCurrentLimit;
        singleJointedArmConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        armTalon1 = new TalonFX(ElevatorConstants.elevatorTalonID);
        armTalon2 = new TalonFX();

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
                arm1Current);
        ParentDevice.optimizeBusUtilizationForAll(elevatorTalon);
    }

    @Override
    public void updateInputs(ModuleIO.ModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus =
                BaseStatusSignal.refreshAll(drivePosition, driveVelocity, arm1AppliedVolts, arm1Current);
        var turnStatus =
                BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
        var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = arm1AppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = arm1Current.getValueAsDouble();

        // Update turn inputs
        inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
        inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
                drivePositionQueue.stream()
                        .mapToDouble((Double value) -> Units.rotationsToRadians(value))
                        .toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream()
                        .map((Double value) -> Rotation2d.fromRotations(value))
                        .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    }
}
