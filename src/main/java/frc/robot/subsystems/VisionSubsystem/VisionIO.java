package frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
        public int[] PhotonTrackedTarget = new int[0];
        public boolean hasEstimate = false;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}

