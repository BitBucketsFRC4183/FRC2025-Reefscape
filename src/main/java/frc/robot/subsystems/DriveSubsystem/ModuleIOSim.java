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
import static frc.robot.constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SparkUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
  private final PIDController turnController = new PIDController(turnSimP, 0, turnSimD);
  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(driveSimKs, driveSimKv, driveSimKa);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor =
            moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(299));
    this.turnMotor =
            moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(300));

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts
              + driveController.calculate(
              moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }

    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(
              moduleSimulation.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec =
            moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps =
            Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));


    // Update odometry inputs
    inputs.odometryPhoenixTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad = Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = driveFF.calculate(velocityRadPerSec);
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}