package frc.robot.subsystems.ArmSubsystem;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.DriveSubsystem.PhoenixOdometryThread;

import java.util.Queue;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ArmIOTalonFX implements ArmIO {
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



    public StatusSignal<Angle> arm1Position;

    public StatusSignal<AngularVelocity> arm1Velocity;
    public StatusSignal<Current> arm1Current;

    private final StatusSignal<Voltage> arm1AppliedVolts;



    public StatusSignal<Angle> arm2Position;

    public StatusSignal<AngularVelocity> arm2Velocity;
    public StatusSignal<Current> arm2Current;

    private final StatusSignal<Voltage> arm2AppliedVolts;

    private final Debouncer arm1ConnectedDebounce = new Debouncer(0.5);
    private final Debouncer arm2ConnectedDebounce = new Debouncer(0.5);


    public ArmIOTalonFX() {

        TalonFXConfiguration arm1Config = new TalonFXConfiguration();
        TalonFXConfiguration arm2Config = new TalonFXConfiguration();

        arm1Config.Audio.BeepOnConfig = false;
        arm1Config.Audio.AllowMusicDurDisable = true;
        arm1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        arm1Config.TorqueCurrent.PeakForwardTorqueCurrent = ArmConstants.arm1CurrentLimit;
        arm1Config.TorqueCurrent.PeakReverseTorqueCurrent = -ArmConstants.arm1CurrentLimit;
        arm1Config.CurrentLimits.StatorCurrentLimit = ArmConstants.arm1CurrentLimit;
        arm1Config.CurrentLimits.StatorCurrentLimitEnable = true;
        arm1Config.MotorOutput.Inverted = ArmConstants.arm1Inverted;

        arm2Config.Audio.BeepOnConfig = false;
        arm2Config.Audio.AllowMusicDurDisable = true;
        arm2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        arm2Config.TorqueCurrent.PeakForwardTorqueCurrent = ArmConstants.arm2CurrentLimit;
        arm2Config.TorqueCurrent.PeakReverseTorqueCurrent = -ArmConstants.arm2CurrentLimit;
        arm2Config.CurrentLimits.StatorCurrentLimit = ArmConstants.arm2CurrentLimit;
        arm2Config.CurrentLimits.StatorCurrentLimitEnable = true;
        arm2Config.MotorOutput.Inverted = ArmConstants.arm2Inverted;


        armTalon1 = new TalonFX(ArmConstants.arm1TalonID);
        armTalon2 = new TalonFX(ArmConstants.arm2TalonID);
        tryUntilOk(5, () -> armTalon1.getConfigurator().apply(arm1Config, 0.25));
        tryUntilOk(5, () -> armTalon2.getConfigurator().apply(arm2Config, 0.25));

        arm1Position = armTalon1.getPosition();
        arm1Velocity = armTalon1.getVelocity();
        arm1AppliedVolts = armTalon1.getMotorVoltage();
        arm1Current = armTalon1.getStatorCurrent();

        arm2Position = armTalon2.getPosition();
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

        Robot.orchestra.addInstrument(armTalon1, 0);
        Robot.orchestra.addInstrument(armTalon2, 1);
    }

    public void updateInputs(ArmIO.ArmIOInputs inputs) {
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
    }

    @Override
    public void setArmMotorVoltage(double volts) {
        armTalon1.setVoltage(volts);
        armTalon2.setVoltage(volts);
    }
}
