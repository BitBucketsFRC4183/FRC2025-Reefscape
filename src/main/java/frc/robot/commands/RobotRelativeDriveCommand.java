package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorInput;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class RobotRelativeDriveCommand extends Command {
        private final DriveSubsystem drive;
        private final DoubleSupplier xSupplier;
        private final DoubleSupplier ySupplier;
        private final DoubleSupplier omegaSupplier;

        public RobotRelativeDriveCommand(DriveSubsystem drive,
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

        double speedFactor;
        if (OperatorInput.alignmentRobotRelative.getAsBoolean()) {
            speedFactor = DriveConstants.alignmentSpeed;
        } else if (OperatorInput.slowModeHold.getAsBoolean()) {
            speedFactor = DriveConstants.slowSpeed;
        } else if (OperatorInput.turboModeHold.getAsBoolean()) {
            speedFactor = DriveConstants.turboSpeed;
        } else {
            speedFactor = DriveConstants.normalSpeed;
        }
        drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * speedFactor,
                        linearVelocity.getY() * speedFactor,
                        omega * drive.getMaxAngularSpeedRadPerSec(),
                        new Rotation2d(0)));
    }

        @Override
        public void end(boolean interrupted) {
            drive.stop();
        }
}
