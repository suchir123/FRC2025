package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.util.NetworkTablesUtil;

public final class LimeLight {
    private static final NetworkTable TABLE = NetworkTablesUtil.getTable("limelight");
    
    private LimeLight() {
    }

    public static void poke() {
        System.out.println("LimeLight initialized");
    }
    
    public static void setLimelightPipeline(int pipeline) {
        TABLE.getEntry("pipeline").setNumber(pipeline);
    }
    
    public static int getLimeLightPipeline() {
        return TABLE.getEntry("pipeline").getNumber(1).intValue();
    }
    
    public static float getLimelightTX() {
        return NetworkTablesUtil.getEntry("limelight", "tx").getNumber(0.0).floatValue();
    }
    
    public static float getLimelightTY() {
        return NetworkTablesUtil.getEntry("limelight", "ty").getNumber(0.0).floatValue();
    }
    
    public static String getLimeyJson() {
        return NetworkTablesUtil.getNTInstance().getTable("limelight").getEntry("json").getString("");
    }
    
    public static int getLimeyTargetTag() {
        return NetworkTablesUtil.getNTInstance().getTable("limelight").getEntry("tid").getNumber(0).intValue();
    }
    
    public static boolean isLimeyConnected() {
        return true; // TODO: impl
    }
    
    public static Pose2d getLimeyApriltagPose() {
        double[] arr = NetworkTablesUtil.getNTInstance().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[] {});
        if(arr.length == 0) return new Pose2d();
        return new Pose2d(arr[0], arr[1], Rotation2d.fromDegrees(arr[5]));
    }
    
    public static double getLimeyTimestamp() {
        return 69; // TODO: impl
    }
    
    public static double getLimeyTagDistance() {
        return 420; // TODO: impl
    }
}