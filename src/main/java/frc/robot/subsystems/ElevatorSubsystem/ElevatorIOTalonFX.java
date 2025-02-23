package frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.DriveSubsystem.ModuleIO;
import frc.robot.subsystems.DriveSubsystem.PhoenixOdometryThread;

import java.util.Queue;

public class ElevatorIOTalonFX implements ElevatorIO{

    private final TalonFX elevatorTalon;

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

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfig.TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.elevatorMotorCurrentLimit;
        elevatorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.elevatorMotorCurrentLimit;
        elevatorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.elevatorMotorCurrentLimit;
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        elevatorTalon = new TalonFX(ElevatorConstants.elevatorCanID);

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

        inputs.odometryPhoenixTimestamps =
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
