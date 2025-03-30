package frc.robot.commands.drive.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.drive.pathfinding.commands.PathfindThenFollowPath2;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.Util;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.function.*;

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
	private static final PathConstraints CONSTRAINTS = new PathConstraints(3, 2, 540, 540, 12);
	private static GoalEndState mostRecentSet = null;
	private static boolean configured = false;
	private static BiFunction<PathPlannerPath, PathConstraints, PathfindThenFollowPath2> pathfindThenFollowPathCommandBuilder;
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
	// General idea:
	// pre-generate enough PathPlanner paths from various start positions -> each reef location
	// During gameplay, driver will set target reef
	// Robot will find the closest path start location, dynamic-path towards that point, then drive to the reef with pre-gen path
	// Ideally, pick a path with a start point that's closer to the reef rather than farther
	// We want to avoid high changes in velocity (both magnitude and direction)
	// We also want to avoid being overly inefficient (trade-off between efficiency and # of pregens)
	
	/**
	 * Configures PathfindingManager for dynamic path following.
	 *
	 * @param poseSupplier                a supplier for the robot's current pose
	 * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
	 *                                    speeds
	 * @param output                      Output function that accepts robot-relative ChassisSpeeds and feedforwards for
	 *                                    each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
	 *                                    using a differential drive, they will be in L, R order.
	 *                                    <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
	 *                                    module states, you will need to reverse the feedforwards for modules that have been flipped
	 * @param controller                  Path following controller that will be used to follow paths
	 * @param robotConfig                 The robot configuration
	 * @param shouldFlipPath              Supplier that determines if paths should be flipped to the other side of
	 *                                    the field. This will maintain a global blue alliance origin.
	 * @param driveRequirements           the subsystem requirements for the robot's drive train
	 * @see AutoBuilder#configure(Supplier, Consumer, Supplier, BiConsumer, PathFollowingController, RobotConfig, BooleanSupplier, Subsystem...)
	 */
	public static void configure(
		Supplier<Pose2d> poseSupplier,
		Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
		BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
		PathFollowingController controller,
		RobotConfig robotConfig,
		BooleanSupplier shouldFlipPath,
		Subsystem... driveRequirements) {
		if (configured) {
			DriverStation.reportError(
				"Auto builder has already been configured. This is likely in error.", true);
		}
		
		configured = true;
		
		pathfindThenFollowPathCommandBuilder =
			(path, constraints) ->
				new PathfindThenFollowPath2(
					path,
					constraints,
					poseSupplier,
					robotRelativeSpeedsSupplier,
					output,
					controller,
					robotConfig,
					shouldFlipPath,
					driveRequirements);
	}
	
	/**
	 * Configures PathfindingManager for dynamic path following.
	 *
	 * @param poseSupplier                a supplier for the robot's current pose
	 * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
	 *                                    speeds
	 * @param output                      Output function that accepts robot-relative ChassisSpeeds.
	 * @param controller                  Path following controller that will be used to follow paths
	 * @param robotConfig                 The robot configuration
	 * @param shouldFlipPath              Supplier that determines if paths should be flipped to the other side of
	 *                                    the field. This will maintain a global blue alliance origin.
	 * @param driveRequirements           the subsystem requirements for the robot's drive train
	 * @see AutoBuilder#configure(Supplier, Consumer, Supplier, Consumer, PathFollowingController, RobotConfig, BooleanSupplier, Subsystem...)
	 */
	public static void configure(
		Supplier<Pose2d> poseSupplier,
		Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
		Consumer<ChassisSpeeds> output,
		PathFollowingController controller,
		RobotConfig robotConfig,
		BooleanSupplier shouldFlipPath,
		Subsystem... driveRequirements) {
		configure(
			poseSupplier,
			robotRelativeSpeedsSupplier,
			(speeds, feedforwards) -> output.accept(speeds),
			controller,
			robotConfig,
			shouldFlipPath,
			driveRequirements);
	}
	
	private static PathfindThenFollowPath2 pathfindThenFollowPath(PathPlannerPath goalPath, PathConstraints pathfindingConstraints) {
		return pathfindThenFollowPathCommandBuilder.apply(goalPath, pathfindingConstraints);
	}
	
	public static void configurePathfinder(Pathfinder p) {
		Pathfinding.setPathfinder(p);
	}
	
	private static Pathfinder getPathfinder() {
		try {
			Field f = Pathfinding.class.getDeclaredField("pathfinder");
			f.setAccessible(true);
			return (Pathfinder) f.get(null);
		} catch (NoSuchFieldException | IllegalAccessException e) {
			throw new RuntimeException(e);
		}
	}
	
	public static PathPlannerPath getNewestPathfindingPath() {
		if (mostRecentSet == null) {
			// System.out.println("No path being run");
			return null;
		}
		return Pathfinding.getCurrentPath(CONSTRAINTS, mostRecentSet);
	}
	
	public static Pose2d extractStartPose(PathPlannerPath p) {
		return p.getStartingHolonomicPose().orElse(p.getStartingDifferentialPose());
	}
	
	private static List<PathPlannerPath> importPaths(List<String> pathNames) {
		List<PathPlannerPath> paths = new ArrayList<>(pathNames.size());
		for (String pathName : pathNames) {
			try {
				paths.add(PathPlannerPath.fromPathFile(pathName));
			} catch (Exception e) {
				System.out.println("Failed to add path " + pathName + " to pathfinding list");
				e.printStackTrace();
			}
		}
		
		return paths;
	}
	
	private static GoalEndState getGoalEndState(PathPlannerPath bestPath) {
		IdealStartingState s = bestPath.getIdealStartingState();
		Rotation2d targetRotation = Rotation2d.kZero;
		if (s == null) {
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
	
	public PathPlannerPath getBestPath(Pose2d currentPose, PathChooser pathChooser) {
		return pathChooser.bestPath(currentPose, this.pathList);
	}
	
	public PathPlannerPath getBestPath(Pose2d currentPose) {
		return this.getBestPath(currentPose, this.pathChooser);
	}
	
	private static PathPlannerTrajectory getPathfindingCommandCurrentTrajectory(PathfindingCommand pfCom) {
		try {
			Field f = PathfindingCommand.class.getDeclaredField("currentTrajectory");
			f.setAccessible(true);
	
			return (PathPlannerTrajectory) f.get(pfCom);
		} catch (NoSuchFieldException | IllegalAccessException e) {
			throw new RuntimeException(e);
		}
	}
	
	/**
	 * @see com.pathplanner.lib.commands.PathfindingCommand#PathfindingCommand(PathPlannerPath, PathConstraints, Supplier, Supplier, BiConsumer, PathFollowingController, RobotConfig, BooleanSupplier, Subsystem...)
	 */
	public Command getFullCommand(Pose2d currentPose, PathChooser h) {
		PathPlannerPath bestPath = this.getBestPath(currentPose, h);
		GoalEndState goalEndState = getGoalEndState(bestPath);
		// System.out.println(goalEndState);
		PathfindThenFollowPath2 p = pathfindThenFollowPath(bestPath, CONSTRAINTS);
		
		return new InstantCommand(() -> mostRecentSet = goalEndState).andThen(p.raceWith(Commands.run(() -> {
			if(p.getPfCom() == null) {
				System.out.println("pfcom null");
			}
			PathPlannerTrajectory ppTraj = getPathfindingCommandCurrentTrajectory(p.getPfCom());
			if(ppTraj != null) {
				DriveTrainSubsystem.pathfinderPathPub.set(Util.convertPPTrajStateListToDoubleArray(ppTraj.getStates()));
				System.out.println("pptraj not null");
			} else {
				// System.out.println("pptraj null");
			}
		})));//.andThen(new InstantCommand(() -> mostRecentSet = null));
	}
	
	public Command getFullCommand(Pose2d currentPose) {
		return this.getFullCommand(currentPose, this.pathChooser);
	}
	
	public void setPreferenceHeuristic(PathChooser h) {
		this.pathChooser = h;
	}
}