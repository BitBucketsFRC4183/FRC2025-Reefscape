package frc.robot.subsystems.VisionSubsystem;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.TimedRobot;
// import ntcore

import edu.wpi.first.math.geometry.*;
//calculate the positions

import java.io.IOException;
import java.util.HashSet; //don't allow replicated vaules
import java.util.LinkedList; //vaule storage
import java.util.List;
import java.util.Set;
// above, data analysis

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

//subsystem setup
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
//aprilTag

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
// getting into photon and position


public class VisionSubsystem extends SubsystemBase {
    // Creates a new ExampleSubsystem
    public final AprilTagFieldLayout aprilTagFieldLayout;
    public final PhotonCamera camera;

    public VisionSubsystem() throws IOException {
        camera = new PhotonCamera("BitbucketCamera");
        aprilTagFieldLayout =
                new AprilTagFieldLayout(AprilTagFields.kDefaultField.m_resourceFile);
    }

    public final Transform3d cameraToRobot =
            new Transform3d();


    @Override
    public void periodic() {
        // get result
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();

        List<PhotonTrackedTarget> targets =
                result.getTargets();
        PhotonTrackedTarget target =
                result.getBestTarget();

        //apriltag
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();


        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
            // data Pose3d robotPose
        };
    }
}
