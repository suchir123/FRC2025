package frc.robot.commands.drive.pathfinding;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;

public interface Pathfinder2 extends Pathfinder {
	PathPlannerPath getCurrentPathWithoutUpdate(PathConstraints constraints, GoalEndState goalEndState);
}
