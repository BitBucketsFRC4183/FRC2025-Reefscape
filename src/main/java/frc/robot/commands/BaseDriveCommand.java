// Copyright 2021-2024 FRC 6328
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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class BaseDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;

    public BaseDriveCommand(DriveSubsystem drive,
                            DoubleSupplier xSupplier,
                            DoubleSupplier ySupplier,
                            DoubleSupplier omegaSupplier) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
    }

    @Override
    public void execute() {
        // Apply deadband
        double DEADBAND = 0.1;
        double linearMagnitude =
                MathUtil.applyDeadband(
                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
        Rotation2d linearDirection =
                new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

        // Square values
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = Math.copySign(omega * omega, omega);

        // Calcaulate new linear velocity
        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

        // Convert to field relative speeds & send command
        boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
        drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega * drive.getMaxAngularSpeedRadPerSec(),
                        isFlipped
                                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                : drive.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
