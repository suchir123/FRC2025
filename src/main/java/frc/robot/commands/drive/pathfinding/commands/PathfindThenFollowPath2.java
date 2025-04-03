package frc.robot.commands.drive.pathfinding.commands;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.pathfinding.PathfindingManager;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.Util;

import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Command group that will pathfind to the start of a path, then follow that path
 */
public class PathfindThenFollowPath2 extends SequentialCommandGroup {
	
	private final PathfindingCommand2 pfCom;
	private final Function<Supplier<PathPlannerPath>, Command> generateDeferredPathJoinerCommand;

	private static boolean f() {
		return false;
	}
	
	/**
	 * Constructs a new PathfindThenFollowPath command group.
	 *
	 * @param goalPath                   the goal path to follow
	 * @param pathfindingConstraints     the path constraints for pathfinding
	 * @param poseSupplier               a supplier for the robot's current pose
	 * @param currentRobotRelativeSpeeds a supplier for the robot's current robot relative speeds
	 * @param output                     Output function that accepts robot-relative ChassisSpeeds and feedforwards for
	 *                                   each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
	 *                                   using a differential drive, they will be in L, R order.
	 *                                   <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
	 *                                   module states, you will need to reverse the feedforwards for modules that have been flipped
	 * @param controller                 Path following controller that will be used to follow the path
	 * @param robotConfig                The robot configuration
	 * @param shouldFlip             Should the target path be flipped to the other side of the field? This
	 *                                   will maintain a global blue alliance origin.
	 * @param requirements               the subsystems required by this command (drive subsystem)
	 */
	public PathfindThenFollowPath2(
		PathPlannerPath goalPath,
		PathConstraints pathfindingConstraints,
		Supplier<Pose2d> poseSupplier,
		Supplier<ChassisSpeeds> currentRobotRelativeSpeeds,
		BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
		PathFollowingController controller,
		RobotConfig robotConfig,
		BooleanSupplier shouldFlip,
		Subsystem... requirements) {
		final BooleanSupplier shouldFlipPath = PathfindThenFollowPath2::f;
		this.generateDeferredPathJoinerCommand = (supJoinTo -> Commands.defer(
			() -> {
				PathPlannerPath joinTo = supJoinTo.get();
				if (joinTo.numPoints() < 2) {
					return Commands.none();
				}
				
				Pose2d startPose = poseSupplier.get();
				ChassisSpeeds startSpeeds = currentRobotRelativeSpeeds.get();
				ChassisSpeeds startFieldSpeeds =
					ChassisSpeeds.fromRobotRelativeSpeeds(startSpeeds, startPose.getRotation());
				Rotation2d startHeading =
					new Rotation2d(
						startFieldSpeeds.vxMetersPerSecond, startFieldSpeeds.vyMetersPerSecond);
				
				Pose2d endWaypoint =
					new Pose2d(joinTo.getPoint(0).position, joinTo.getInitialHeading());
				boolean shouldFlipP = shouldFlipPath.getAsBoolean() && !joinTo.preventFlipping;
				if (shouldFlipP) {
					endWaypoint = FlippingUtil.flipFieldPose(endWaypoint);
				}
				
				GoalEndState endState;
				if (joinTo.getIdealStartingState() != null) {
					Rotation2d endRot = joinTo.getIdealStartingState().rotation();
					if (shouldFlipP) {
						endRot = FlippingUtil.flipFieldRotation(endRot);
					}
					endState = new GoalEndState(joinTo.getIdealStartingState().velocityMPS(), endRot);
				} else {
					endState =
						new GoalEndState(
							pathfindingConstraints.maxVelocityMPS(), startPose.getRotation());
				}
				
				PathPlannerPath joinPath =
					new PathPlannerPath(
						PathPlannerPath.waypointsFromPoses(
							new Pose2d(startPose.getTranslation(), startHeading), endWaypoint),
						pathfindingConstraints,
						new IdealStartingState(
							Math.hypot(startSpeeds.vxMetersPerSecond, startSpeeds.vyMetersPerSecond),
							startPose.getRotation()),
						endState);
				joinPath.preventFlipping = true;
				
				return new FollowPathCommand(
					joinPath,
					poseSupplier,
					currentRobotRelativeSpeeds,
					output,
					controller,
					robotConfig,
					shouldFlipPath,
					requirements);
			},
			Set.of(requirements)));
		
		// ? generate end path except longer
		// ? generate follow path to there?
		// use those paths to find the point we want
		
		// 1. find intermediate point
		// 2. generate intermediate path between intermediate point and start of goal path
		
		// need to generate intermediate path here
		// then just change the goal path in this.pfCom to that instead
		
		// we need to extend the points on the end path as part of the connection algo
		List<Pose2d> pathPoses = goalPath.getPathPoses();
		Pose2d goalPathStart = new Pose2d(pathPoses.get(0).getTranslation(), goalPath.getIdealStartingState().rotation());
		boolean extendable = pathPoses.size() > 1 && poseSupplier.get().getTranslation().getDistance(goalPathStart.getTranslation()) > 0.5;
		if (extendable) {
			System.out.println("extendable, going for it");
			Rotation2d slopeStart = Util.slopeAngle(goalPathStart, pathPoses.get(1)); // slope
			System.out.println("GPS: " + goalPathStart + ", slope: " + slopeStart);
			Pose2d extended = new Pose2d(goalPathStart.getTranslation().plus(new Translation2d(0.5, slopeStart)), goalPathStart.getRotation()); // more
			System.out.println("Extended pose: " + extended);
			
			this.pfCom = new PathfindingCommand2( // path find to the extended part of the path
				extended,
				pathfindingConstraints,
				goalPath.getIdealStartingState().velocity(),
				poseSupplier,
				currentRobotRelativeSpeeds,
				output,
				controller,
				robotConfig,
				requirements
			);
			
			/*
			 * The new target point to end the dynamic pathplanning portion at.
			 */
			AtomicReference<Pose2d> newTarget = new AtomicReference<>();
			
			/*
			 * The path connecting the new target point to the beginning of the preplanned path
			 */
			AtomicReference<PathPlannerPath> connectorPath = new AtomicReference<>();
			
			/*
			 * A command to follow the connectorPath
			 */
			AtomicReference<Command> connectorCommand = new AtomicReference<>();
			addCommands(
				// the pfCom will pathfind to a "farther away" point from where it should
				// its targetPose is also public
				// we just need to make another command here that runs alongside pfCom that changes targetPose to be the start point of the next connection path
				this.pfCom.alongWith(Commands.waitSeconds(0.25).andThen(Commands.runOnce(() -> {
					System.out.println("RUNNING THE RUNONCE PART");
					// get the currently running path
					PathPlannerPath pathfindingPath = PathfindingManager.getNewestPathfindingPath(); // this shouldn't screw anything up
					if (pathfindingPath != null) {
						System.out.println("found a pathfinding path");
						List<Pose2d> poses = pathfindingPath.getPathPoses(); // you guessed it. another slope calculation
						if (poses.size() > 1) {
							// as a YOLO heuristic (read: i'm coding this at 5am) we can just kinda guess where we wanna end the previous path. maybe 0.5m before it ends?
							// if the path is less than 0.5m total then just do nothing, since either 1) we started out close anyways, or 2) we were previously doing this already so just stick to it
							final double tooFarAway = 1;
							if (poses.get(poses.size() - 1).getTranslation().getDistance(poses.get(0).getTranslation()) > tooFarAway) {
								System.out.println("far enough away, finding a new target point");
								// sample it at 0.5m before the end
								Pose2d last = poses.get(poses.size() - 1);
								int idx = poses.size() - 1;
								// iterate from the end until we find a pose 0.5m away
								for (int i = poses.size() - 2; i >= 0; i--) {
									if (poses.get(i).getTranslation().getDistance(last.getTranslation()) > tooFarAway) {
										System.out.println("found the first one that's at the distance limit. idx: " + i + ", pose: " + poses.get(i) + ", poses size: " + poses.size());
										idx = i;
										break;
									}
								}
								Rotation2d slope = Util.slopeAngle(poses.get(idx), poses.get(idx + 1)); // slope of the cut-off area of auto-gen path
								// with the slope we want to use it to generate a smooth bezier trajectory
								// note: in a PathPlannerPath the Rotation2d is the heading of the trajectory, NOT of the robot chassis. trajectory heading = the direction of the robot's velocity vector
								newTarget.set(new Pose2d(poses.get(idx).getTranslation(), slope.plus(Rotation2d.k180deg)));
								System.out.println("new target: " + newTarget);
								System.out.println("goal end rotation for pathfinding: " + goalPath.getIdealStartingState().rotation());
								this.pfCom.targetPose = newTarget.get();
								// Pathfinding.setGoalPosition(newTarget.get().getTranslation()); // sneak in and change it
							} else {
								newTarget.set(poses.get(poses.size() - 1)); // just go to endpoint
							}
							
							if (Util.isSim()) {
								RobotContainer.INSTANCE.driveTrain.setPose(new Pose2d(newTarget.get().getTranslation(), goalPath.getIdealStartingState().rotation()));
							}
						}
					}
				}))),
				
				// Use a deferred command to generate an on-the-fly path to join
				// the end of the pathfinding command to the start of the path
				generateDeferredPathJoinerCommand.apply(() -> {
					Pose2d p = newTarget.get();
					if (p != null) {
						System.out.println("new target exists");
						Pose2d end = new Pose2d(goalPathStart.getTranslation(), slopeStart.plus(Rotation2d.k180deg)); // the original start of pre-planned paths
						System.out.println("end of path joiner: " + end);
						var waypoints = PathPlannerPath.waypointsFromPoses(
							new Pose2d(poseSupplier.get().getTranslation(), newTarget.get().getRotation()), // the cut off part from the end of pathfinding
							end
						);
						connectorPath.set(new PathPlannerPath(waypoints, pathfindingConstraints, goalPath.getIdealStartingState(), new GoalEndState(goalPath.getIdealStartingState().velocityMPS(), goalPath.getIdealStartingState().rotation())));
						return connectorPath.get();
					}
					System.out.println("new target doesn't exist");
					return goalPath;
				}),
				
				// Note: I think this is like that coconut file in that one video game where if you delete it then nothing works
				// Because I don't even think the deferred command gets run
				// like it throws an error every time
				Commands.defer(() -> {
						var cPath = connectorPath.get();
						if (cPath != null) {
							var poses = cPath.getPathPoses();
							if (poses.size() > 1 && poses.get(0).getTranslation().getDistance(poses.get(poses.size() - 1).getTranslation()) > 0.75) {
								System.out.println("connector path exists & is long enough, connecting...");
								connectorCommand.set(new FollowPathCommand(
									cPath,
									poseSupplier,
									currentRobotRelativeSpeeds,
									output,
									controller,
									robotConfig,
									shouldFlip,
									requirements));
								System.out.println("made the connector command");
							}
						} else {
							System.out.println("connector path doesn't exist");
							connectorCommand.set(new InstantCommand(() -> System.out.println("connector path null, skipping connector path.")));
						}
						return connectorCommand.get();
					}, Set.of(requirements)
				).alongWith(new InstantCommand(() -> {
					if (connectorCommand.get() instanceof FollowPathCommand fpc) {
						DriveTrainSubsystem.connectionPathPub.set(Util.convertPPTrajStateListToDoubleArray(trajFromFollowPathCom(fpc).getStates()));
					}
				})),
				
				new FollowPathCommand( // follow pre-planned path
					goalPath,
					poseSupplier,
					currentRobotRelativeSpeeds,
					output,
					controller,
					robotConfig,
					shouldFlip,
					requirements)
			);
		} else {
			System.out.println("not extendable, doing it the normal way");
			this.pfCom = new PathfindingCommand2(
				goalPath,
				pathfindingConstraints,
				poseSupplier,
				currentRobotRelativeSpeeds,
				output,
				controller,
				robotConfig,
				shouldFlip,
				requirements);
			
			addCommands(
				this.pfCom,
				// Use a deferred command to generate an on-the-fly path to join
				// the end of the pathfinding command to the start of the path
				generateDeferredPathJoinerCommand.apply(() -> goalPath),
				
				new FollowPathCommand(
					goalPath,
					poseSupplier,
					currentRobotRelativeSpeeds,
					output,
					controller,
					robotConfig,
					shouldFlip,
					requirements));
		}
	}
	
	private static PathPlannerTrajectory trajFromFollowPathCom(FollowPathCommand c) {
		try {
			var f = FollowPathCommand.class.getDeclaredField("trajectory");
			f.setAccessible(true);
			return (PathPlannerTrajectory) f.get(c);
		} catch (NoSuchFieldException | IllegalAccessException e) {
			throw new RuntimeException(e);
		}
	}
	
	public PathfindingCommand2 getPfCom() {
		return this.pfCom;
	}
}
