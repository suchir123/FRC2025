package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * A class to manage selecting the best path to follow in order to reach a desired target.
 * The path's start point will be routed to using dynamic pathfinding, meaning that the selection of the best path should hinge on selecting the most optimal pre-planned path to seek towards.
 */
public class PathfindingManager {
	/**
	 * A default heuristic for path selection preference.
	 * Based on {@link Pose2d#nearest(List)}
	 */
	private static final PathChooser defaultChooser = (currentPose, paths) -> Collections.min(
		paths,
		Comparator.<PathPlannerPath, Double>comparing( // sob emoji. https://stackoverflow.com/questions/24436871/very-confused-by-java-8-comparator-type-inference
			(pathPlannerPath -> currentPose.getTranslation().getDistance(extractStartPose(pathPlannerPath).getTranslation()))
		).thenComparing(
			(pathPlannerPath -> currentPose.getRotation().minus(extractStartPose(pathPlannerPath).getRotation()).getRadians())
		)
	);
	
	public static Pose2d extractStartPose(PathPlannerPath p) {
		return p.getStartingHolonomicPose().orElse(p.getStartingDifferentialPose());
	}
	
	// private static final PathPreferenceHeuristic defaultHeuristic = ((currentPose, pathStartPose, pathEndPose) -> 999999 - (Math.pow((pathStartPose.getX() - currentPose.getX()), 2) + Math.pow((pathStartPose.getY() - currentPose.getY()), 2)));
	private static final PathConstraints CONSTRAINTS = new PathConstraints(3, 2, 540, 540, 12);
	// General idea:
	// pre-generate enough PathPlanner paths from various start positions -> each reef location
	// During gameplay, driver will set target reef
	// Robot will find the closest path start location, dynamic-path towards that point, then drive to the reef with pre-gen path
	// Ideally, pick a path with a start point that's closer to the reef rather than farther
	// We want to avoid high changes in velocity (both magnitude and direction)
	// We also want to avoid being overly inefficient (trade-off between efficiency and # of pregens)
	
	private static List<PathPlannerPath> importPaths(List<String> pathNames) {
		List<PathPlannerPath> paths = new ArrayList<>(pathNames.size());
		for (String pathName : pathNames) {
			try {
				paths.add(PathPlannerPath.fromPathFile(pathName));
			} catch(Exception e) {
				System.out.println("Failed to add path " + pathName + " to pathfinding list");
				e.printStackTrace();
			}
		}
		
		return paths;
	}
	
	private final List<PathPlannerPath> pathList;
	private PathChooser pathChooser;
	
	private PathfindingManager(List<PathPlannerPath> pathList, PathChooser pathChooser, Void ignored) {
		this.pathList = pathList;
		this.pathChooser = pathChooser;
	}
	
	public PathfindingManager(List<String> pathNameList, PathChooser pathChooser) {
		this(importPaths(pathNameList), pathChooser, null);
	}
	
	public PathfindingManager(List<String> pathNameList) {
		this(pathNameList, defaultChooser);
	}
	
	public PathPlannerPath getBestPath(Pose2d currentPose, PathChooser pathChooser) {
		return pathChooser.bestPath(currentPose, this.pathList);
	}
	
	public PathPlannerPath getBestPath(Pose2d currentPose) {
		return this.getBestPath(currentPose, this.pathChooser);
	}
	
	public Command getFullCommand(Pose2d currentPose, PathChooser h) {
		return AutoBuilder.pathfindThenFollowPath(this.getBestPath(currentPose, h), CONSTRAINTS);
	}
	
	public Command getFullCommand(Pose2d currentPose) {
		return this.getFullCommand(currentPose, this.pathChooser);
	}
	
	public void setPreferenceHeuristic(PathChooser h) {
		this.pathChooser = h;
	}
}