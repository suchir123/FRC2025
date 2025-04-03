package frc.robot.commands.drive.pathfinding;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

@FunctionalInterface
public interface PathChooser {
	/**
	 * Note: the given pose is field-relative, but the pre-planned paths will only be for one side of the field. YOU WILL BE GIVEN PRE-FLIPPED PATHS IF NECESSARY. DO NOT DOUBLE-FLIP.
	 * Note 2: RETURN THE FLIPPED PATH YOU ARE GIVEN.
	 * @param currentPose The robot's current field pose
	 * @param paths List of paths to choose from
	 * @return The best path to follow
	 */
	PathPlannerPath bestPath(Pose2d currentPose, List<PathPlannerPath> paths);
}
