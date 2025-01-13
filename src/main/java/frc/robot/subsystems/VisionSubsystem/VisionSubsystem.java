package frc.robot.subsystems.VisionSubsystem;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.TimedRobot;
// import ntcore

import edu.wpi.first.math.geometry.*;
//calculate the positions

import java.io.IOException;
import java.util.*;
// above, data analysis

import edu.wpi.first.math.geometry.Pose3d;

//subsystem setup
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
//aprilTag

import frc.robot.subsystems.DriveSubsystem.GyroIO;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
// getting into photon and position


public class VisionSubsystem extends SubsystemBase {
    public static PhotonPipelineResult visionResult;
    public static Optional<EstimatedRobotPose> etimatedRobotPose;
    // Creates a new ExampleSubsystem
    public AprilTagFieldLayout aprilTagFieldLayout;
    public final String cameraName = "BitbucketCamera";
    public final PhotonCamera camera;
    public final PhotonPoseEstimator photonPoseEstimator;
    public final Transform3d cameraToRobot =
            new Transform3d();
    public final Transform3d fieldToCamera =
            new Transform3d();

    public VisionSubsystem() {
        camera = new PhotonCamera(cameraName);
        try {
            aprilTagFieldLayout =
                    new AprilTagFieldLayout(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);

    }


    @Override
    public void periodic() {
        // get result
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
        }

        Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update(visionResult);
    }
}


