package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.VisionSubsystem.VisionUtil;

public class VisionConstants {
    public static AprilTagFieldLayout aprilTagFieldLayout = VisionUtil.getAprilTagFieldLayoutSafe();

    public static String cameraName =
            "BitbucketCamera";
    public static Transform3d cameraToRobot =
            new Transform3d();

}

