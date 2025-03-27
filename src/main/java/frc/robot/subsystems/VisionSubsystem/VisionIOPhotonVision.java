package frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.constants.VisionConstants;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
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
        inputs.connected = camera.isConnected();

        if (!inputs.driverMode) {

            Set<Short> tagIds = new HashSet<>();
            List<PoseObservation> poseObservations = new LinkedList<>();

            for (var result : camera.getAllUnreadResults()) {
                List<PhotonTrackedTarget> visionTargets =
                        result.getTargets();
                if (result.hasTargets()) {
                    inputs.latestTargetObservation =
                            new TargetObservation(
                                    Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                                    Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
                } else {
                    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
                }

                // Add pose observation
                if (result.multitagResult.isPresent()) { // Multitag result
                    var multitagResult = result.multitagResult.get();

                    PhotonTrackedTarget targets =
                            result.getBestTarget();

                    var tagPose =
                            aprilTagFieldLayout.getTagPose(targets.fiducialId);


                    // Calculate robot pose
                    Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                    Transform3d fieldToRobot =
                            fieldToCamera.plus(robotToCamera1.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Calculate average tag distance
                    double totalTagDistance = 0.0;
                    for (var target : result.targets) {
                        totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                    }

                    // Add tag IDs
                    tagIds.addAll(multitagResult.fiducialIDsUsed);

                    // Add observation
                    poseObservations.add(
                            new PoseObservation(
                                    result.getTimestampSeconds(), // Timestamp
                                    robotPose, // 3D pose estimate
                                    multitagResult.estimatedPose.ambiguity, // Ambiguity
                                    multitagResult.fiducialIDsUsed.size(), // Tag count
                                    totalTagDistance / result.targets.size()// Average tag distance
                            )); // Observation type

                } else if (!result.targets.isEmpty()) { // Single tag result
                    var target = result.targets.get(0);

                    // Calculate robot pose
                    var tagPose =
                            aprilTagFieldLayout.getTagPose(target.fiducialId);
                    if (tagPose.isPresent()) {
                        Transform3d fieldToTarget =
                                new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                        Transform3d cameraToTarget = target.bestCameraToTarget;
                        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                        Transform3d fieldToRobot =
                                fieldToCamera.plus(robotToCamera1.inverse());
                        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                        // Add tag ID
                        tagIds.add((short) target.fiducialId);

                        // Add observation
                        poseObservations.add(
                                new PoseObservation(
                                        result.getTimestampSeconds(), // Timestamp
                                        robotPose, // 3D pose estimate
                                        target.poseAmbiguity, // Ambiguity
                                        1, // Tag count
                                        cameraToTarget.getTranslation().getNorm() // Average tag distance
                                )); // Observation type
                    }
                }

                var optionalPose =
                        photonPoseEstimator.update(result);
                optionalPose.ifPresent(estimatedRobotPose -> inputs.estimatedRobotPose = estimatedRobotPose.estimatedPose);
                optionalPose.ifPresent(estimatedRobotPose -> inputs.timestampSeconds = estimatedRobotPose.timestampSeconds);

                inputs.connected =
                        camera.isConnected();
                inputs.hasEstimate =
                        optionalPose.isPresent();
            }

            // Save pose observations to inputs object
            inputs.poseObservations = new PoseObservation[poseObservations.size()];
            for (int i = 0; i < poseObservations.size(); i++) {
                inputs.poseObservations[i] = poseObservations.get(i);
            }

            // Save tag IDs to inputs objects
            inputs.tagIds = new int[tagIds.size()];
            int i = 0;
            for (int id : tagIds) {
                inputs.tagIds[i++] = id;


            }
        }

    }

    @Override
    public void setDriverMode(boolean isOn) {
        camera.setDriverMode(true);

        Thread m_visionThread;

        m_visionThread = new Thread(() -> {


            UsbCamera camera = CameraServer.startAutomaticCapture();
            camera.setResolution(640, 480);

            //cv stuffs
            CvSink cvSink = CameraServer.getVideo();
            CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

            Mat mat = new Mat();

            while (!Thread.interrupted()) {

                if (cvSink.grabFrame(mat) == 0) {
                    // Send the output the error.
                    outputStream.notifyError(cvSink.getError());
                    // skip the rest of the current iteration
                    continue;
                }

                Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);

                outputStream.putFrame(mat);
            }
        });
        m_visionThread.setDaemon(true);
        m_visionThread.start();
    }
}