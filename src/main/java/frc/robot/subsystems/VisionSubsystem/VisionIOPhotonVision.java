package frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

import static frc.robot.constants.VisionConstants.aprilTagFieldLayout;
import static frc.robot.constants.VisionConstants.robotToCamera1;
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
            Transform3d fieldToCamera = multitagResult.estimatedPose.best;
            Transform3d fieldToRobot =
                    fieldToCamera.plus(robotToCamera1.inverse());
            Pose3d robotPose =
                    new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());}


        if (!hasTargets) {
            targetVisible = true;
            inputs.latestTargetObservation =
                    new TargetObservation(
                            Rotation2d.fromDegrees(visionResult.getBestTarget().getYaw()),
                            Rotation2d.fromDegrees(visionResult.getBestTarget().getPitch()));

        }


        if (!targets.isEmpty()) {
            PhotonTrackedTarget target =
                    visionResult.getBestTarget();

            //tagID
            int targetID = target.getFiducialId();


            //tagPose
            var tagPose = aprilTagFieldLayout.getTagPose(target.fiducialId);
            if (tagPose.isPresent()) {
                Transform3d fieldToTarget =
                        new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());

                Transform3d cameraToTarget = target.bestCameraToTarget;

                Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera1.inverse());

                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                inputs.latestTargetObservation = new TargetObservation(Rotation2d.fromDegrees(target.getYaw()),
                        Rotation2d.fromDegrees(target.getPitch()));

                inputs.tagPose =
                        aprilTagFieldLayout.getTagPose(targetID).get();
            }

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