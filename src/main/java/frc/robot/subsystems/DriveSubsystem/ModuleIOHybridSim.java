// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.DriveSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.DriveSubsystem.ModuleIOHybrid;
import frc.robot.subsystems.DriveSubsystem.ModuleIOTalonFX;
import frc.robot.util.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control.
 */
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
    private final SimulatedMotorController.GenericMotorController turnSpark;
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

    private final SwerveModuleSimulation simulation;

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


    public ModuleIOHybrid(int module, SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants, SwerveModuleSimulation simulation) {
        this.constants = constants;
        this.simulation = simulation;
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
        driveConfig.Slot0 = new Slot0Configs().withKP(driveSimP).withKD(driveSimD).withKV(driveSimKv).withKA(driveSimKa).withKS(driveSimKs);
        driveConfig.Feedback.SensorToMechanismRatio = driveMotorReduction;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveMotorCurrentLimit;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveMotorCurrentLimit;
        driveConfig.CurrentLimits.StatorCurrentLimit = driveMotorCurrentLimit;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.Feedback.FeedbackRotorOffset = 0;

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

        this.turnFF = new SimpleMotorFeedforward(0, DriveConstants.turnFF, 0);
        // SparkUtil.tryUntilOk(turnSpark, 5, () -> turnEncoder.setPosition(thrifty.getRadians() - zeroRotation.getRadians()));
        turnController = new ProfiledPIDController(turnKp, 0, turnKd, new TrapezoidProfile.Constraints(9999, 9999));
        turnController.enableContinuousInput(turnPIDMinInput, turnPIDMaxInput);

        simulation.useSteerMotorController(new SimulatedMotorController.GenericMotorController(DCMotor.getNEO(1)));
        simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));


    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signal
        var driveStatus =
                BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
        // if v = 0 lol
        // inputs.turnEncoderConnected = turnEncoder.getConnected();
        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();


        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnPosition = simulation.getSteerAbsoluteFacing();
        inputs.turnVelocityRadPerSec =
                simulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = simulation.getSteerMotorAppliedVoltage().in(Volts);
        inputs.turnCurrentAmps =
                Math.abs(simulation.getSteerMotorStatorCurrent().in(Amps));

        // Update odometry inputs
        inputs.odometryPhoenixTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
        inputs.odometryDrivePositionsRad = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Radians))
                .toArray();
        inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();


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
        turnSpark.requestVoltage(Volts.of(output));
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
                        simulation.getSteerAbsoluteFacing().getRadians(), turnPIDMinInput, turnPIDMaxInput);


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
