package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class FieldRelativeDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;

    private final Supplier<Rotation2d> headingSupplier;

    public FieldRelativeDriveCommand(DriveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, Supplier<Rotation2d> headingSupplier) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.headingSupplier = headingSupplier;
    }


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
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;


        ChassisSpeeds speeds_robotOriented =  new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(), //4.5 is the experimentally determined max velocity
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega * Math.PI * 1.5
        );

        Rotation2d headingFlipped = headingSupplier.get();

        if (isFlipped) {
            headingFlipped = headingFlipped.plus(Rotation2d.fromDegrees(180));
        }

        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds_robotOriented, headingFlipped));
    }




    public void end(boolean interrupted) {
        drive.stop();
    }
}
