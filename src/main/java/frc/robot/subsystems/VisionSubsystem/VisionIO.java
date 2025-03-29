package frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public Pose3d estimatedRobotPose = new Pose3d();
        public boolean hasEstimate = false;
        public double timestampSeconds;
        public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
        public Pose3d tagPose;
        public TargetObservation targetID() {
            return latestTargetObservation;
        }
        public int[] tagIds = new int[0];
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public boolean driverMode = false;

    }
    public static record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance) {}



    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}
    public default void updateInputs(VisionIOInputs inputs) {}
    public default void setDriverMode(boolean isOn) {}
}

