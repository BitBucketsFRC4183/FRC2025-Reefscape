package frc.robot.subsystems.ElevatorSubsystem;

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
import frc.robot.constants.ElevatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.ModuleIO;
import frc.robot.subsystems.DriveSubsystem.PhoenixOdometryThread;

import java.util.Queue;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ElevatorIOTalonFX implements ElevatorIO{

    private TalonFX elevatorTalon;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
            new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
            new VelocityTorqueCurrentFOC(0.0);

    public Queue<Double> timestampQueue;

    public static Queue<Double> elevatorPositionQueue;

    public StatusSignal<Angle> elevatorPosition;

    public StatusSignal<AngularVelocity> elevatorVelocity;
    public StatusSignal<Current> elevatorCurrent;

    private final StatusSignal<Voltage> elevatorAppliedVolts;
    private final Debouncer elevatorConnectedDebounce = new Debouncer(0.5);


    public ElevatorIOTalonFX() {

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        var elevatorConfig = talonFXConfiguration;

        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfig.TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.elevatorMotorCurrentLimit;
        elevatorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.elevatorMotorCurrentLimit;
        elevatorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.elevatorMotorCurrentLimit;
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        elevatorTalon = new TalonFX(ElevatorConstants.elevatorTalonID);

            timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

            elevatorPosition = elevatorTalon.getPosition();
            elevatorPositionQueue =
                    PhoenixOdometryThread.getInstance().registerSignal(elevatorTalon.getPosition());
            elevatorVelocity = elevatorTalon.getVelocity();
            elevatorAppliedVolts = elevatorTalon.getMotorVoltage();
            elevatorCurrent = elevatorTalon.getStatorCurrent();


            BaseStatusSignal.setUpdateFrequencyForAll(
                    50.0,
                    elevatorPosition,
                    elevatorVelocity,
                    elevatorAppliedVolts,
                    elevatorCurrent);
            ParentDevice.optimizeBusUtilizationForAll(elevatorTalon);
        }

        public void updateInputs(ModuleIO.ModuleIOInputs inputs) {

            var elevatorStatus =
                    BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity, elevatorAppliedVolts, elevatorCurrent);


            ElevatorConstants.elevatorConnected = elevatorConnectedDebounce.calculate(elevatorStatus.isOK());
            ElevatorConstants.elevatorPositionRad = Units.rotationsToRadians(elevatorPosition.getValueAsDouble());
            ElevatorConstants.elevatorVelocityRadPerSec = Units.rotationsToRadians(elevatorVelocity.getValueAsDouble());
            ElevatorConstants.elevatorAppliedVoltage = elevatorAppliedVolts.getValueAsDouble();
            ElevatorConstants.elevatorCurrentAmps = elevatorCurrent.getValueAsDouble();

            inputs.odometryTimestamps =
                    timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
            inputs.odometryDrivePositionsRad =
                    elevatorPositionQueue.stream()
                            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
                            .toArray();
            timestampQueue.clear();
            elevatorPositionQueue.clear();
        }


        public void setElevatorVelocity(double velocityRadPerSec) {
            double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
            elevatorTalon.set(0);
        }




}
