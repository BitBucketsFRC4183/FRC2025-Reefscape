package frc.robot.subsystems.VisionSubsystem;


import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
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

        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));

        cameraProp.setCalibError(0.25, 0.08);

        cameraProp.setFPS(20);

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
        
        boolean targetVisible = false;

        var visionResult =
                camera.getLatestResult();
        boolean hasTargets =
                visionResult.hasTargets();

        if (!hasTargets){
        targetVisible = true;}


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


        var debugField = visionSim.getDebugField();
        debugField.getObject("EstimatedRobot").setPose(DriveSubsystem.getPose());

        visionSim.getDebugField();
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);

    }

}