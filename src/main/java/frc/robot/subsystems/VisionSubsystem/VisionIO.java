package frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public Pose3d estimatedRobotPose;
        public boolean hasEstimate = false;
        public double timestampSeconds;
        public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
        public Pose3d tagPose;
        public TargetObservation targetID() {
            return latestTargetObservation;
        }
    }

    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}
    public default void updateInputs(VisionIOInputs inputs) {}
}

