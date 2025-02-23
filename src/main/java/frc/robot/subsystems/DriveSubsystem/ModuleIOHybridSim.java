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

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SparkUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import java.util.Arrays;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveConstants.*;

/** Physics sim implementation of module IO. */
public class ModuleIOHybridSim extends ModuleIOHybrid {
  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private boolean turnClosedLoop = false;
  private final PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
  private final PIDController turnController = new PIDController(turnSimP, 0, turnSimD);

  private double turnAppliedVolts = 0.0;

  public ModuleIOHybridSim(int module, SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants, SwerveModuleSimulation simulation) {
    super(module,constants);
    this.moduleSimulation = simulation;
    this.driveMotor = simulation.useDriveMotorController(
            new PhoenixUtil.TalonFXMotorControllerSim(driveTalon, constants.DriveMotorInverted));
    this.turnMotor =
            moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(turnMotorCurrentLimit));

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    super.updateInputs(inputs);
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));
    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec =
            moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps =
            Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

    // Update odometry inputs
    inputs.odometryPhoenixTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad = Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
  }


  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }


  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}