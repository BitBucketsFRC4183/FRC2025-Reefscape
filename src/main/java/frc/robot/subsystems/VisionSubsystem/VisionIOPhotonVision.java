package frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.TimedRobot;
// import ntcore

import edu.wpi.first.math.geometry.*;
//calculate the positions

// above, data analysis

//subsystem setup

//aprilTag

import org.photonvision.*;
// getting into photon and position





public class VisionIOPhotonVision implements VisionIO {
    public final PhotonCamera camera;
    public PhotonPoseEstimator photonPoseEstimator;

    public VisionIOPhotonVision() {
        this.camera =
                new PhotonCamera(VisionConstants.camera1Name);
        photonPoseEstimator =
                new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotToCamera1);

    }


    public void updateInputs(VisionIOInputs inputs) {
        var visionResult = camera.getLatestResult();

        List<PhotonTrackedTarget> targets =
                visionResult.getTargets();

        boolean hasTargets =
                visionResult.hasTargets();
        boolean targetVisible = false;

        if (visionResult.multitagResult.isPresent()) { //
            var multitagResult =
                    visionResult.multitagResult.get();

        if (!hasTargets) {
            targetVisible = true;
        }


        if (!targets.isEmpty()) {
            PhotonTrackedTarget target =
                    visionResult.getBestTarget();

            //apriltag
            int targetID = target.getFiducialId();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

            inputs.latestTargetObservation = new TargetObservation(Rotation2d.fromDegrees(target.getYaw()), Rotation2d.fromDegrees(target.getPitch()));
            inputs.tagPose =
                    VisionConstants.aprilTagFieldLayout.getTagPose(targetID).get();
        }

        Transform3d fieldToCamera = multitagResult.estimatedPose.best;

        var optionalPose = photonPoseEstimator.update(visionResult);
        optionalPose.ifPresent(estimatedRobotPose -> inputs.estimatedRobotPose = estimatedRobotPose.estimatedPose);
        optionalPose.ifPresent(estimatedRobotPose -> inputs.timestampSeconds = estimatedRobotPose.timestampSeconds);

        inputs.connected =
                camera.isConnected();
        inputs.hasEstimate =
                optionalPose.isPresent();

        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);

        }
    }
}