package frc.robot.commands.drive.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A class to manage selecting the best path to follow in order to reach a desired target.
 * The path's start point will be routed to using dynamic pathfinding, meaning that the selection of the best path should hinge on selecting the most optimal pre-planned path to seek towards.
 */
public class PathfindingManager {
	private static GoalEndState mostRecentSet = null;
	
	public static PathPlannerPath getNewestPathfindingPath() {
		if(mostRecentSet == null) {
			// System.out.println("No path being run");
			return null;
		}
		return Pathfinding.getCurrentPath(CONSTRAINTS, mostRecentSet);
	}
	
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
	
	/**
	 *
	 * @see com.pathplanner.lib.commands.PathfindingCommand#PathfindingCommand(PathPlannerPath, PathConstraints, Supplier, Supplier, BiConsumer, PathFollowingController, RobotConfig, BooleanSupplier, Subsystem...)
	 */
	public Command getFullCommand(Pose2d currentPose, PathChooser h) {
		PathPlannerPath bestPath = this.getBestPath(currentPose, h);
		GoalEndState goalEndState = getGoalEndState(bestPath);
		// System.out.println(goalEndState);
		return new InstantCommand(() -> {
			mostRecentSet = goalEndState;
			System.out.println("setting most recent\nsetting most recent\nsetting most recent\nsetting most recent\nsetting most recent\nsetting most recent\nsetting most recent\nsetting most recent\n");
		}).andThen(AutoBuilder.pathfindThenFollowPath(bestPath, CONSTRAINTS));//.andThen(new InstantCommand(() -> mostRecentSet = null));
	}
	
	private static GoalEndState getGoalEndState(PathPlannerPath bestPath) {
		IdealStartingState s = bestPath.getIdealStartingState();
		Rotation2d targetRotation = Rotation2d.kZero;
		if(s == null) {
			for (PathPoint p : bestPath.getAllPathPoints()) {
				if (p.rotationTarget != null) {
					targetRotation = p.rotationTarget.rotation();
					break;
				}
			}
		} else {
			targetRotation = bestPath.getIdealStartingState().rotation();
		}
		return new GoalEndState(bestPath.getGlobalConstraints().maxVelocityMPS(), targetRotation);
	}
	
	public Command getFullCommand(Pose2d currentPose) {
		return this.getFullCommand(currentPose, this.pathChooser);
	}
	
	public void setPreferenceHeuristic(PathChooser h) {
		this.pathChooser = h;
	}
}