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

import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.GyroIO;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
// getting into photon and position


public class VisionSubsystem extends SubsystemBase {
    // Creates a new ExampleSubsystem
    public AprilTagFieldLayout aprilTagFieldLayout;
    public final Transform3d cameraToRobot =
            new Transform3d();

    private final VisionIO visionIO;
    private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();


    public VisionSubsystem(VisionIO visionIO) {
        this.visionIO = visionIO;

    }


    @Override
    public void periodic() {
        visionIO.updateInputs(visionInputs);
    }

    public Pose3d getEstimatedRobotPose() {
        return visionInputs.estimatedRobotPose;
    }
}


