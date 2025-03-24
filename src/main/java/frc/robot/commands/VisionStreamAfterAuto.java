package frc.robot.commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem.VisionIO;
import frc.robot.subsystems.VisionSubsystem.VisionIOPhotonVision;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;


public class VisionStreamAfterAuto extends Command {
    private PhotonCamera camera;


    public VisionStreamAfterAuto(){
        this.camera =
                new PhotonCamera(VisionConstants.camera1Name);
    }

    @Override
    public void execute() {
        camera.setDriverMode(true);

        Thread m_visionThread;

        CameraServer.startAutomaticCapture();

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

    public void updateInputs(VisionIO.VisionIOInputs inputs) {
        inputs.setDriverMode = true;
    }

}
