package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;
import java.util.Map;

public class PathfindingManager {
	/**
	 * A default heuristic for path selection preference. It only takes into account the Euclidean distance between the two poses (no preference for going towards the end goal, or for rotation targets).
	 */
	private static final PathPreferenceHeuristic defaultHeuristic = ((currentPose, pathStartPose, pathEndPose) -> Math.pow((pathStartPose.getX() - currentPose.getX()), 2) + Math.pow((pathStartPose.getY() - currentPose.getY()), 2));
	private static final PathConstraints CONSTRAINTS = new PathConstraints(3, 2, 540, 540, 12);
	// General idea:
	// pre-generate enough PathPlanner paths from various start positions -> each reef location
	// During gameplay, driver will set target reef
	// Robot will find the closest path start location, dynamic-path towards that point, then drive to the reef with pre-gen path
	// Ideally, pick a path with a start point that's closer to the reef rather than farther
	// We want to avoid high changes in velocity (both magnitude and direction)
	// We also want to avoid being overly inefficient (trade-off between efficiency and # of pregens)
	
	/**
	 * A map of reef indices (1 -> 6) to a list of paths
	 */
	private final Map<Integer, List<PathPlannerPath>> reefIndexToPathList;
	private PathPreferenceHeuristic preferenceHeuristic;
	
	public PathfindingManager(Map<Integer, List<PathPlannerPath>> reefIndexToPathList, PathPreferenceHeuristic h) {
		this.reefIndexToPathList = reefIndexToPathList;
		this.preferenceHeuristic = h;
	}
	
	public PathfindingManager(Map<Integer, List<PathPlannerPath>> reefIndexToPathList) {
		this(reefIndexToPathList, defaultHeuristic);
	}
	
	public List<PathPlannerPath> getPathsForReef(int reefIdx) {
		return reefIndexToPathList.get(reefIdx);
	}
	
	public PathPlannerPath getPathForReef(int reefIdx, Pose2d currentPose, PathPreferenceHeuristic heuristic) {
		List<PathPlannerPath> paths = getPathsForReef(reefIdx);
		PathPlannerPath bestPath = null;
		double bestScore = Double.NEGATIVE_INFINITY;
		for(PathPlannerPath path : paths) {
			Pose2d pose = path.getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
			if(pose != null) {
				List<Pose2d> pathPoses = path.getPathPoses();
				double score = heuristic.score(currentPose, pose, pathPoses.get(pathPoses.size() - 1));
				if(score > bestScore) {
					bestScore = score;
					bestPath = path;
				}
			}
		}
		return bestPath;
	}
	
	public PathPlannerPath getPathForReef(int reefIdx, Pose2d currentPose) {
		return this.getPathForReef(reefIdx, currentPose, this.preferenceHeuristic);
	}
	
	public Command getCommandForReef(int reefIdx, Pose2d currentPose, PathPreferenceHeuristic h) {
		return AutoBuilder.pathfindThenFollowPath(getPathForReef(reefIdx, currentPose, h), CONSTRAINTS);
	}
	
	public Command getCommandForReef(int reefIdx, Pose2d currentPose) {
		return this.getCommandForReef(reefIdx, currentPose, this.preferenceHeuristic);
	}
	
	public void setPreferenceHeuristic(PathPreferenceHeuristic h) {
		this.preferenceHeuristic = h;
	}
	
	@FunctionalInterface
	public interface PathPreferenceHeuristic {
		double score(Pose2d currentPose, Pose2d pathStartPose, Pose2d pathEndPose);
	}
}