package frc.robot.subsystems.VisionSubsystem;

import frc.robot.constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.TimedRobot;
// import ntcore

import edu.wpi.first.math.geometry.*;
//calculate the positions

import java.io.IOException;
import java.util.*;
// above, data analysis

//subsystem setup
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
//aprilTag

import org.photonvision.*;
// getting into photon and position





public class VisionIOPhotonVision implements VisionIO {
    public final PhotonCamera camera;
    public final PhotonPoseEstimator photonPoseEstimator;

    public VisionIOPhotonVision() {
        this.camera =
                new PhotonCamera(VisionConstants.cameraName);
        photonPoseEstimator =
                new PhotonPoseEstimator(VisionConstants.aprilTagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.cameraToRobot);

    }


    public void updateInputs(VisionIOInputs inputs) {
        var visionResult = camera.getLatestResult();
        boolean hasTargets =
                visionResult.hasTargets();

        List<PhotonTrackedTarget> targets =
                visionResult.getTargets();

        if (!targets.isEmpty()) {
            PhotonTrackedTarget target =
                    visionResult.getBestTarget();

            //apriltag
            int targetID = target.getFiducialId();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

        }

        inputs.estimatedPose = photonPoseEstimator.update(visionResult);
        inputs.connected =
                camera.isConnected();

        inputs.PhotonTrackedTarget = targets;
        inputs.hasEstimate =
                inputs.estimatedPose.isPresent();
    }
}