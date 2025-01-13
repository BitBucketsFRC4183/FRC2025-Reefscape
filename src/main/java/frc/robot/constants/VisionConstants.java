package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public static AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static String cameraName =
            "BitbucketCamera";
    public static Transform3d cameraToRobot =
            new Transform3d();



}

