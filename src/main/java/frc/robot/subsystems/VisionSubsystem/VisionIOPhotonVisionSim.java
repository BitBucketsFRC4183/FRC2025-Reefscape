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

        // initial
        this.poseSupplier = poseSupplier;
        if (visionSim == null) {
            visionSim = new VisionSystemSim(
                    "VisionSubsystem");
            visionSim.addAprilTags(VisionConstants.aprilTagFieldLayout);
        }


        // Add sim camera
        //properties

        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(20);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);


        cameraSim = new PhotonCameraSim(new PhotonCamera("simCamera")
                        , cameraProp);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        camera = cameraSim.getCamera();
        
        visionSim.addCamera(cameraSim, VisionConstants.robotToCamera1);
        photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotToCamera1);

        var debugField = visionSim.getDebugField();
        visionSim.getDebugField();

    }


    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
        }
    }