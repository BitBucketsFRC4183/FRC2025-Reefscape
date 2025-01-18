package frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.UncheckedIOException;

public class VisionUtil {
    public static AprilTagFieldLayout getAprilTagFieldLayoutSafe() {
        try {
            return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e){
            try {
                return AprilTagFieldLayout.loadFromResource("./2025-reefscape.json");
            } catch (IOException e2) {
                throw new RuntimeException();
            }
        }
    };
}
