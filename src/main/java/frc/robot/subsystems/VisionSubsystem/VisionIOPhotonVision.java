package frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

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

        Set<Short> tagIds = new HashSet<>();
        List<PhotonTrackedTarget> targets =
                visionResult.getTargets();
        List<PoseObservation> poseObservations = new LinkedList<>();

        boolean hasTargets =
                visionResult.hasTargets();
        boolean targetVisible = false;

        //has targets
        if (!hasTargets) {
            targetVisible = true;
            inputs.latestTargetObservation =
                    new TargetObservation(
                            Rotation2d.fromDegrees(visionResult.getBestTarget().getYaw()),
                            Rotation2d.fromDegrees(visionResult.getBestTarget().getPitch()));
        } else {
        inputs.latestTargetObservation =
                new TargetObservation(new Rotation2d(), new Rotation2d());}

        //no targets
        if (!targets.isEmpty()) {
            PhotonTrackedTarget target =
                    visionResult.getBestTarget();}



        if (visionResult.multitagResult.isPresent()) { //
             //multitags
            var multitagResult =
                    visionResult.multitagResult.get();

            //relative poses
            Transform3d fieldToCamera = multitagResult.estimatedPose.best;
            Transform3d fieldToRobot =
                    fieldToCamera.plus(robotToCamera1.inverse());

            //robot pose accordingly to multitags
            Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

            tagIds.addAll(multitagResult.fiducialIDsUsed);

            //tag distance, used for
            // pose observation
            double totalTagDistance = 0.0;
            for (var target : visionResult.targets) {
                totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
            }

            poseObservations.add(
                    new PoseObservation(
                            visionResult.getTimestampSeconds(),
                            robotPose,
                            multitagResult.estimatedPose.ambiguity,
                            multitagResult.fiducialIDsUsed.size(),
                            totalTagDistance / visionResult.targets.size())
            );}

        // If no multitags
            else if (!visionResult.targets.isEmpty()) { //
                // Single tag result
                var target = visionResult.targets.get(0);

            //utilize Pose
            var tagPose =
                    VisionConstants.aprilTagFieldLayout.getTagPose(target.fiducialId);
            if (tagPose.isPresent()) {

                //relatives to the target
                Transform3d fieldToTarget =
                        new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                Transform3d cameraToTarget = target.bestCameraToTarget;

                //relatives to the field to
                // robot in order to get the
                // robot pose
                Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                Transform3d fieldToRobot =
                        fieldToCamera.plus(robotToCamera1.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                //pose observation without
                // multitags
                poseObservations.add(
                        new PoseObservation(
                                visionResult.getTimestampSeconds(),
                                robotPose,
                                target.poseAmbiguity,
                                1,
                                cameraToTarget.getTranslation().getNorm()));
            }


            inputs.poseObservations = new PoseObservation[poseObservations.size()];

            inputs.latestTargetObservation = new TargetObservation(Rotation2d.fromDegrees(target.getYaw()),
                    Rotation2d.fromDegrees(target.getPitch()));

//            inputs.tagPose =
//                    aprilTagFieldLayout.getTagPose(tagIds).get();

            inputs.tagIds = new int[tagIds.size()];

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