package frc.robot.subsystems.VisionSubsystem;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.function.Supplier;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;
    private final PhotonCamera camera;
    private final Supplier<Pose2d> poseSupplier;

    public VisionIOPhotonVisionSim(Supplier<Pose2d> poseSupplier) {
        super();
        // Initialize vision sim
        this.poseSupplier = poseSupplier;
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(VisionConstants.aprilTagFieldLayout);
        }

        // Add sim camera
        var cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
// Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
// Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
// The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim =
                new PhotonCameraSim(new PhotonCamera("simCamera")
                , cameraProp);
        camera = cameraSim.getCamera();

        visionSim.addCamera(cameraSim, VisionConstants.cameraToRobot);
        photonPoseEstimator =
                new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.cameraToRobot);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);

        var visionResult =
                camera.getLatestResult();
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

            inputs.latestTargetObservation = new TargetObservation(Rotation2d.fromDegrees(target.getYaw()), Rotation2d.fromDegrees(target.getPitch()));
            inputs.tagPose =
                    VisionConstants.aprilTagFieldLayout.getTagPose(targetID).get();
        }

        var optionalPose = photonPoseEstimator.update(visionResult);
        optionalPose.ifPresent(estimatedRobotPose -> inputs.estimatedRobotPose = estimatedRobotPose.estimatedPose);
        optionalPose.ifPresent(estimatedRobotPose -> inputs.timestampSeconds = estimatedRobotPose.timestampSeconds);

        inputs.connected =
                camera.isConnected();
        inputs.hasEstimate =
                optionalPose.isPresent();
    }
}