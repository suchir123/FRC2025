package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.NetworkTablesUtil;

public final class QuestNav {
	public static final QuestNav INSTANCE = new QuestNav();
	private Rotation2d gyroResetAngle = new Rotation2d();
	// Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
	NetworkTable nt4Table = NetworkTablesUtil.getTable("questnav");
	private final IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
	private final IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();
	// Subscribe to the Network Tables questnav data topics
	private final DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
	private final FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
	private final FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
	private final FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
	private final DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);
	// Local heading helper variables
	private float yaw_offset = 0.0f;
	private Translation2d questNavRawToFieldCoordinateSystem = new Translation2d();
	
	private QuestNav() {}
	
	// Gets the Quest's measured position.
	public Pose2d getPose() {
		// System.out.println("QNR -> FCS: " + questNavRawToFieldCoordinateSystem);

		return new Pose2d(getQuestNavPose().getTranslation().rotateBy(gyroResetAngle.plus(Rotation2d.k180deg)).plus(questNavRawToFieldCoordinateSystem), getOculusRot2d());
	}
	
	// Gets the battery percent of the Quest.
	public double getBatteryPercent() {
		return questBatteryPercent.get();
	}
	
	// Returns if the Quest is connected.
	public boolean connected() {
		return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
	}
	
	// Gets the Quaternion of the Quest.
	public Quaternion getQuaternion() {
		float[] qqFloats = questQuaternion.get();
		return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
	}
	
	// Gets the Quests's timestamp.
	public double timestamp() {
		return questTimestamp.get();
	}
	
	// Clean up questnav subroutine messages after processing on the headset
	public void cleanUpQuestNavMessages() {
		if (questMiso.get() == 99) {
			questMosi.set(0);
		}
	}
	
	// Get the yaw Euler angle of the headset
	private float getOculusYaw() {
		float[] eulerAngles = questEulerAngles.get();
		var ret = eulerAngles[1] - yaw_offset;
		ret %= 360;
		if (ret < 0) {
			ret += 360;
		}
		return ret;
	}
	
	private Translation2d getQuestNavTranslation() {
		float[] questnavPosition = questPosition.get();
		return new Translation2d(questnavPosition[2], -questnavPosition[0]);
	}

	private Rotation2d getOculusRot2d() {
		return Rotation2d.kZero.minus(Rotation2d.fromDegrees(getOculusYaw())).plus(gyroResetAngle);
	}
	
	private Pose2d getQuestNavPose() {
		var oculousPositionCompensated = getQuestNavTranslation(); // 6.5
		return new Pose2d(oculousPositionCompensated, getOculusRot2d());
	}
	
	public void resetPose(Pose2d pose) {
		questMosi.accept(3);
		
		questNavRawToFieldCoordinateSystem =
			pose.getTranslation()
				.minus(getPose().getTranslation().minus(questNavRawToFieldCoordinateSystem));
	}
	
	public void resetHeading(Rotation2d heading) {
		questMosi.accept(3);
		
		gyroResetAngle =
			(getPose().getRotation().minus(gyroResetAngle).minus(heading))
				.unaryMinus();
	}
	
	public void resetHeading() {
		resetHeading(
			DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.kPi : Rotation2d.kZero);
	}
	
	public void resetBlue() {
		resetHeading(Rotation2d.kZero);
	}
}