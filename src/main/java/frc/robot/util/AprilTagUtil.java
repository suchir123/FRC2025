package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

public final class AprilTagUtil {
    private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static Optional<Pose3d> getTagPose(int tagId) {
        return FIELD_LAYOUT.getTagPose(tagId);
    }
    
    private AprilTagUtil() {}
}
