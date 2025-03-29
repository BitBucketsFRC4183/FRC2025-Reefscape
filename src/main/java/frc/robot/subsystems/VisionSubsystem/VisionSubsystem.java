package frc.robot.subsystems.VisionSubsystem;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.TimedRobot;
// import ntcore

import edu.wpi.first.math.Matrix;
//calculate the positions

import java.util.*;
// above, data analysis

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;

//subsystem setup
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
//aprilTag

import frc.robot.constants.VisionConstants;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

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
        }
    }

    public void setDriverCameraModeOn() {
        visionIO.setDriverMode(true);
        visionInputs.driverMode = true;
    }

    public Optional<Pose3d> getEstimatedRobotPose() {
        if (visionInputs.poseObservations.length == 0) {return Optional.empty();}
        VisionIO.PoseObservation obv = visionInputs.poseObservations[0];
        if ((obv.ambiguity() < 0.2)) {
            Logger.recordOutput("Odometry/filteredVisionPose", visionInputs.estimatedRobotPose);
            return Optional.of(visionInputs.estimatedRobotPose);
        } else {
            return Optional.empty();
        }
    }

    public boolean hasEstimatedRobotPose() {
        return (visionInputs.hasEstimate && visionInputs.connected);
    }

    public Matrix<N3, N1> getAdjustedStdDevs() {
        if (visionInputs.poseObservations.length == 0) {return new Matrix<>(Nat.N3(), Nat.N1());}
        double amb = visionInputs.poseObservations[0].ambiguity();
        Matrix<N3, N1> stdDevs = new Matrix<>(Nat.N3(), Nat.N1());
        double xDev = amb * VisionConstants.visionTrustFactor;
        double yDev = amb  * VisionConstants.visionTrustFactor;
        double tDev = amb * 0.00;
        stdDevs.set(0, 0, xDev);
        stdDevs.set(1, 0, yDev);
        stdDevs.set(2, 0, tDev);
        return stdDevs;
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

