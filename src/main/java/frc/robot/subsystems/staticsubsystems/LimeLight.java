package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.NetworkTablesUtil;

public final class LimeLight {
    public record LimeyApriltagReading(boolean exists, Pose2d pose, int tag, double distance, double timestamp) {
    }

    private static final NetworkTable TABLE = NetworkTablesUtil.getTable("limelight");
    private static final IntegerSubscriber HEARTBEAT_ENTRY = TABLE.getIntegerTopic("hb").subscribe(-1);
    private static final DoubleArraySubscriber APRILTAG_POSE_ENTRY = TABLE.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    
    private LimeLight() {
    }

    public static void poke() {
        System.out.println("LimeLight initialized");
    }
    
    public static void setLimeyPipeline(int pipeline) {
        TABLE.getEntry("pipeline").setNumber(pipeline);
    }
    
    public static int getLimeyPipeline() {
        return TABLE.getEntry("pipeline").getNumber(1).intValue();
    }
    
    public static float getLimeyTX() {
        return NetworkTablesUtil.getEntry("limelight", "tx").getNumber(0.0).floatValue();
    }
    
    public static float getLimeyTY() {
        return NetworkTablesUtil.getEntry("limelight", "ty").getNumber(0.0).floatValue();
    }
    
    public static LimeyApriltagReading getLimeyApriltagReading() {
        double[] arr = APRILTAG_POSE_ENTRY.get();
        if(arr.length == 0 || !isLimeyConnected()) return new LimeyApriltagReading(false, new Pose2d(), -1, -1, RobotController.getFPGATime());
        int tID = getLimeyTargetTag();

        return new LimeyApriltagReading(tID != -1, new Pose2d(arr[0], arr[1], Rotation2d.fromDegrees(arr[5])), tID, arr[9], APRILTAG_POSE_ENTRY.getLastChange() - arr[6]);
    }
    
    public static int getLimeyTargetTag() {
        return NetworkTablesUtil.getNTInstance().getTable("limelight").getEntry("tid").getNumber(0).intValue();
    }

    public static boolean isLimeyConnected() {
        return ((RobotController.getFPGATime() - HEARTBEAT_ENTRY.getLastChange()) / 1000) < 250;
    }
}