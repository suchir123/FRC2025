package frc.robot.commands.drive.pathfinding;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

/**
 * Note: This implementation WILL handle field flipping. The supplied heuristic has no need to flip any paths.
 */
public class HeuristicBasedPathChooser implements PathChooser {
	private final PathPreferenceHeuristic prefHeuristic;
	public HeuristicBasedPathChooser(PathPreferenceHeuristic prefHeuristic) {
		this.prefHeuristic = prefHeuristic;
	}
	
	@Override
	public PathPlannerPath bestPath(Pose2d currentPose, List<PathPlannerPath> paths) {
		PathPlannerPath bestPath = null;
		double bestScore = Double.NEGATIVE_INFINITY;
		for(PathPlannerPath path : paths) {
			Pose2d pose = PathfindingManager.extractStartPose(path);
			if(pose != null) {
				List<Pose2d> pathPoses = path.getPathPoses();
				double score = prefHeuristic.score(currentPose, pose, pathPoses.get(pathPoses.size() - 1));
				System.out.println("Scored " + path.name + " as " + score);
				if(score > bestScore) {
					bestScore = score;
					bestPath = path;
				}
			}
		}
		return bestPath;
	}
	
	@FunctionalInterface
	public interface PathPreferenceHeuristic {
		double score(Pose2d currentPose, Pose2d pathStartPose, Pose2d pathEndPose);
	}
}
