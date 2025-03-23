package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.VisionSubsystem.VisionUtil;

public class VisionConstants {
    public static AprilTagFieldLayout aprilTagFieldLayout = VisionUtil.getAprilTagFieldLayoutSafe();

    public static String camera0Name =
            "BitbucketDriver'sCamera";
    public static String camera1Name =
            "BitbucketPhotonVision'sCamera";
   //Robot Dimentions: (w=68.55) cm
    // camera to edge of width of the robot: 5.45
    // camera to center of the robot: 28.825cm
    // ->0.28825 m
    //length: 68.58cm
// ----------camera--

    public static Transform3d robotToCamera1 =
            new Transform3d(0.28825, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));



}

