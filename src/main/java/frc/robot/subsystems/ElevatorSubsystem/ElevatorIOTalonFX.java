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
import edu.wpi.first.math.MathUtil;
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
import frc.robot.util.PhoenixUtil;

import java.util.Queue;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

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


        elevatorPosition = elevatorTalon.getPosition();
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
        PhoenixUtil.tryUntilOk(5, () -> elevatorTalon.setPosition(0));

    }

        public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
            var elevatorStatus =
                    BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity, elevatorAppliedVolts, elevatorCurrent);
            inputs.elevatorConnected = elevatorConnectedDebounce.calculate(elevatorStatus.isOK());
            inputs.elevatorMotorPositionRad = Units.rotationsToRadians(elevatorPosition.getValueAsDouble());
            inputs.elevatorMotorVelocityRadPerSec = Units.rotationsToRadians(elevatorVelocity.getValueAsDouble());
            inputs.elevatorAppliedVolts = elevatorAppliedVolts.getValueAsDouble();
            inputs.elevatorCurrentAmps = elevatorCurrent.getValueAsDouble();
        }

        @Override
        public void setElevatorMotorVoltage(double volts) {
            double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
            elevatorTalon.set(appliedVolts);
        }

        @Override
        public void setEncoderHeightValue(double position) {
            PhoenixUtil.tryUntilOk(5, () -> elevatorTalon.setPosition(position));
        }



}
