package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.util.NetworkTablesUtil;

public final class RoboGUI {
	private RoboGUI() {}
	
	private static final NetworkTableEntry placementPositionEntry = NetworkTablesUtil.getEntry("robogui", "selectedPlacementPosition");
	
	public static int getPressedTargetReef() {
		return 5;
		// return placementPositionEntry.getNumber(-1).intValue();
	}
	
	public static void resetPressedTargetReef() {
		placementPositionEntry.setNumber(-1);
	}
}
