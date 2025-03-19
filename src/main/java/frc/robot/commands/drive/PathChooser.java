package frc.robot.commands.drive;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

@FunctionalInterface
public interface PathChooser {
	PathPlannerPath bestPath(Pose2d currentPose, List<PathPlannerPath> paths);
}
