package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorInput;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class FieldDriveElevatorLimitedCommand extends Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;
    private final ElevatorSubsystem elevator;
    private final Supplier<Rotation2d> headingSupplier;

    public FieldDriveElevatorLimitedCommand(DriveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, Supplier<Rotation2d> headingSupplier, ElevatorSubsystem elevatorSubsystem) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.headingSupplier = headingSupplier;
        this.elevator = elevatorSubsystem;
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


        double speedFactor;
        if (OperatorInput.slowModeHold.getAsBoolean()) {
            speedFactor = DriveConstants.slowSpeed;
        } else if (OperatorInput.turboModeHold.getAsBoolean()) {
            speedFactor = DriveConstants.turboSpeed;
        } else {
            speedFactor = DriveConstants.normalSpeed;
        }

        double radFactor;
        if (OperatorInput.slowModeHold.getAsBoolean()) {
            radFactor = 0.333;
        } else if (OperatorInput.turboModeHold.getAsBoolean()) {
            radFactor = 0.75;
        } else {
            radFactor = 0.5;
        }

        double rampFactor = speedRamp(elevator.getLoadHeight(),
                ElevatorConstants.minHeight + 0.25, ElevatorConstants.maxHeight, 0.15, 1, true);

        speedFactor = speedFactor * rampFactor;

        ChassisSpeeds speeds_robotOriented =  new ChassisSpeeds(
                linearVelocity.getX() * speedFactor, //4.5 is the experimentally determined max velocity
                linearVelocity.getY() * speedFactor,
                omega * drive.getMaxAngularSpeedRadPerSec() * rampFactor * radFactor
        );

        Rotation2d headingFlipped = headingSupplier.get();
        if (isFlipped) {
            headingFlipped = headingFlipped.plus(Rotation2d.fromDegrees(180));
        }


        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds_robotOriented, headingFlipped));
    }


    // returns a ramped speed value
    // a value between an min max, gets mapped to a different linear scale
    private double speedRamp(double value, double inputMin, double inputMax, double outputMin, double outputMax, boolean flipped) {
        double frvalue = MathUtil.clamp( value, inputMin, inputMax);
        double range1 = inputMax - inputMin;
        double diff1 = frvalue - inputMin;

        double firstScalar = diff1 / range1;
        if(flipped) {
            firstScalar = 1 - firstScalar;
        }
        double range2 = outputMax - outputMin;
        return range2 * firstScalar + outputMin;
    };

    public void end(boolean interrupted) {
        drive.stop();
    }
}
