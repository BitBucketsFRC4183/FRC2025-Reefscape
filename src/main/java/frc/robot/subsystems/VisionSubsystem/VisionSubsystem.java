package frc.robot.subsystems.VisionSubsystem;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.TimedRobot;
// import ntcore

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.*;
//calculate the positions

import java.io.IOException;
import java.util.*;
// above, data analysis

import edu.wpi.first.math.geometry.Pose3d;

//subsystem setup
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
//aprilTag

import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.GyroIO;
import org.littletonrobotics.junction.Logger;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// getting into photon and position


public class VisionSubsystem extends SubsystemBase {

    public AprilTagFieldLayout aprilTagFieldLayout;

    private final VisionIO visionIO;
    private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

    public VisionSubsystem(VisionIO visionIO) {
        this.visionIO = visionIO;

    }



    @Override
    public void periodic() {
        visionIO.updateInputs(visionInputs);
        Logger.processInputs("VisionSubsystem", visionInputs);
        if (!visionInputs.connected) {
            new Alert("Vision camera " + VisionConstants.camera1Name +
                    "is disconnected.", Alert.AlertType.kWarning);
            Pose3d tagPose;
            tagPose = visionInputs.tagPose;
        }
    }

    public void setDriverCameraModeOn() {
        visionIO.setDriverMode(true);
    }

    public Pose3d getEstimatedRobotPose() {
        return visionInputs.estimatedRobotPose;
    }

    public boolean hasEstimatedRobotPose() {
        return visionInputs.hasEstimate;
    }

    public double getPoseTimestamp() {
        return  visionInputs.timestampSeconds;
    }


}
//** .......................,,-~*~,,
//......................./:.:.:.:.:.|
//......................|;.;.;.;.;./
//......................|.;.;.;.;.|
//............._,,,,,_.).;.;.;.;.|
//.........,,-":.:.:.:."~-,;.;.;.|
//........(_,,,,---,,_:.:.);.;.;..",,
//......,-":.:.:.:.:.""-,,/;.;.;.;.;.",
//.....(:.__,,,,,,,,,___);.;.;.;.;.;|
//...../"":.:.:.:.:.:.:¯""\;.;.;.;.;.,"
//....\",__,,,,,,,,,,,__/;;;;;;;;;/\
//.....\.::.:.:.:.:.:.:.;.);;;;;;;;;/:\
//.......\,,,,,---~~~~;;;;;;;;,"::::\
//.........."""~~--,,,,,,,,,,-"::::::::::\
//...................\::::::::::::::::::::::://

