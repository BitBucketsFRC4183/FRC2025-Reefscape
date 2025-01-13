package frc.robot.subsystems.VisionSubsystem;

import frc.robot.subsystems.DriveSubsystem.GyroIO;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class VisionIOPhoton implements VisionIO {
    public final PhotonCamera camera;

    public VisionIOPhoton(PhotonCamera camera) {
        this.camera = camera;
    }

    List<PhotonTrackedTarget> targets =
            VisionSubsystem.visionResult.getTargets();

    public void updateInputs(VisionIO.VisionIOInputs inputs) {
        inputs.estimatedPose =
                VisionSubsystem.etimatedRobotPose;
        inputs.connected =
                camera.isConnected();

        // inputs.PhotonTrackedTarget = targets
        inputs.hasEstimate =
                VisionSubsystem.etimatedRobotPose.isPresent();
    }
}