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

package frc.robot.constants;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import static edu.wpi.first.units.Units.*;

public class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.2;
    public static final double turboSpeed =  2.5;
    public static final double normalSpeed = 1.5;
    public static final double slowSpeed = 0.5;
    public static final double alignmentSpeed = 0.2;

    public static final double odometryFrequency = 100.0; // Hz

    // update this
    public static final double trackWidth = Units.inchesToMeters(27);
    public static final double wheelBase = Units.inchesToMeters(27);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
            new Translation2d[] {
                    new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                    new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                    new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                    new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
            };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(Math.PI - 3.136);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(Math.PI -0.389);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI - 2.354);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI - 2.014);

    // Device CAN IDs
    public static final int pigeonCanId = 11;

    public static final int frontLeftDriveCanId = 8;
    public static final int backLeftDriveCanId = 9;
    public static final int frontRightDriveCanId = 6;
    public static final int backRightDriveCanId = 7;

    public static final int frontLeftTurnCanId = 5;
    public static final int backLeftTurnCanId = 4;
    public static final int frontRightTurnCanId = 2;
    public static final int backRightTurnCanId = 3;

    // Encoder Ports

    public static final int frontLeftEncoderPort = 2;
    public static final int frontRightEncoderPort = 3;
    public static final int backLeftEncoderPort = 1;
    public static final int backRightEncoderPort = 0;


    // Drive motor configuration
    public static final boolean driveInverted = false;

    // stator is for peaks, neo 40A breakers can sustain 120 easily, and 70 for more than like 20seconds
    public static final int driveMotorStatorCurrentLimit = 120;
    public static final int driveMotorSupplyCurrentLimit = 70;
    public static final double wheelRadiusMeters = 0.0508;
    public static final double driveMotorReduction = 6.75;
    public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
            2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
            (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive Velocity firmware PID gains
    public static final double driveKp = 0.17423;
    public static final double driveKd = 0.0;
    public static final double driveKa = 0.0038184;
    public static final double driveKs = 0.1884;
    public static final double driveKv = 0.77595;

    // tune kV till perfect
    // kS measured through sysid, + a lil extra
    // kA and KP jack it up until oscillation
    // https://www.reca.lc/motors
    // for krakens: 4200 rpm, 0.94 torque, at 50A, this is to make sure acceleration and over current stuff does not occur
    // better to be conservative
    // essentially to tune run motor at 12V or as close as possible, then find a good enough traj that matches max accel
    // adjust max speed . motor max speed to whatever value that the cutoff curve does not fit
    public static final double driveSimP = 3.75;
    public static final double driveSimD = 0.00;
    public static final double driveSimKs = 0.029022;
    public static final double driveSimKv = 0.88;
    public static final double driveSimKa = 0.00998804;

    // Turn motor configuration
    public static final boolean turnInverted = true;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 12.8;
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    // Turn encoder configuration
    public static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec
    public static final boolean turnEncoderInverted = true;

    // Turn PID configuration
    public static final double turnKp = 11.0;
    public static final double turnFF = 1.3;
    public static final double turnKd = 0.2;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = -Math.PI; // Radians
    public static final double turnPIDMaxInput = Math.PI; // Radians

    public static final double slewX = 99;
    public static final double slewY = 99;
    public static final double slewTheta = 99;

    public static final double robotMassLb = 120.00;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig =
            new RobotConfig(
                    Units.lbsToKilograms(robotMassLb),
                    robotMOI,
                    new ModuleConfig(
                            wheelRadiusMeters,
                            maxSpeedMetersPerSec,
                            wheelCOF,
                            driveGearbox.withReduction(driveMotorReduction),
                            driveMotorStatorCurrentLimit,
                            1),
                    moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(moduleTranslations)
            .withRobotMass(Pounds.of(robotMassLb))
            .withGyro(COTS.ofPigeon2())
            .withBumperSize(Inches.of(15), Inches.of(15))
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    driveGearbox,
                    turnGearbox,
                    driveMotorReduction,
                    turnMotorReduction,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    Meters.of(wheelRadiusMeters),
                    KilogramSquareMeters.of(0.02),
                    wheelCOF));
    public static SwerveModuleConstants.ClosedLoopOutputType DriveMotorClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;

    // HolonomicDrivePID
    public static final double kXHoloP = 0;
    public static final double kXHoloI = 0;
    public static final double kXHoloD = 0;

    public static final double kYHoloP = 0;
    public static final double kYHoloI = 0;
    public static final double kYHoloD = 0;

    public static final double kTHoloP = 0;
    public static final double kTHoloI = 0;
    public static final double kTHoloD = 0;


    public static final double kXHoloPSim = 8;
    public static final double kXHoloISim = 0;
    public static final double kXHoloDSim = 0.1;

    public static final double kYHoloPSim = 8;
    public static final double kYHoloISim = 0;
    public static final double kYHoloDSim = 0.1;

    public static final double kTHoloPSim = 3;
    public static final double kTHoloISim = 0;
    public static final double kTHoloDSim = 0.0;

    public static final double kHoloXTolerance = 0.01;
    public static final double kHoloYTolerance = 0.02;
    public static final double kHoloTTolerance = Units.degreesToRadians(5);
    public static final double maxAngularSpeedRadPerSecAuto = 6.826;
    public static final double maxAngularAccelRadPerSecSquareAuto = 38.523;
}