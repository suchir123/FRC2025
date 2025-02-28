package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Flags;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.AprilTagUtil;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.Util;

public class ReefAprilTagCenterCommand extends Command {
    public static final double MAX_SPEED_METERS_PER_SEC = Flags.DriveTrain.LOWER_MAX_SPEED ? 1.5 : 3;
    public static final double MAX_ROT_SPEED_ANGULAR = 3;
    private final DriveTrainSubsystem driveTrain;
    private final AbstractController joystick;
    // private final AprilTagHandler aprilTagHandler;
    // private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(1);
    // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);
    // private final Trigger autoAimSubwoofer;
    // private final LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    // private boolean wasAutomaticallyDrivingLastFrame = false;

    public ReefAprilTagCenterCommand(DriveTrainSubsystem driveTrain, AbstractController joystick) { //AprilTagHandler aprilTagHandler) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;
        // this.aprilTagHandler = aprilTagHandler;
        // this.autoAimSubwoofer = ControlHandler.get(joystick, ControllerConstants.AUTO_AIM_FOR_SHOOT);

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        // this.driveTrain.setPose(new Pose2d(2, 7, RobotGyro.getRotation2d()));
    }

    private double flipFactor() {
        if (Util.onBlueTeam()) {
            return 1;
        } else {
            return -1;
        }
    }

    @Override
    public void execute() {
        if(Util.onBlueTeam()) {
            NetworkTablesUtil.setLimelightPipeline(1);
        } else {
            NetworkTablesUtil.setLimelightPipeline(0);
        }
        double flip = flipFactor();
        // System.out.println("vert: " + this.joystick.getRightVerticalMovement() + ", hor: " + this.joystick.getRightHorizontalMovement());
        // this.driveTrain.drive(this.joystick.getVerticalMovement());
        final double kPTranslation = 0.1;
        final double kPRotation = 0.1;
        //double flip = flipFactor();
        double pixelDiff = -NetworkTablesUtil.getLimelightTX();
        int tagId = NetworkTablesUtil.getLimeyTargetTag();
        double ySpeedError = -Util.squareKeepSign(this.ySpeedLimiter.calculate(this.joystick.getLeftVerticalMovement() * flip)) * MAX_SPEED_METERS_PER_SEC;
        double xSpeedError = MathUtil.clamp(kPTranslation * pixelDiff, -1, 1);
        // System.out.println("xSpeed = " + xSpeed);
        // System.out.println("ySpeed = " + ySpeed);

        Optional<Pose3d> tagPose = AprilTagUtil.getTagPose(tagId);
        double rotSpeed = 0;
        if(tagPose.isPresent()) {
            Rotation2d targetAngle = AprilTagUtil.getTagPose(tagId).orElseGet(Pose3d::new).getRotation().toRotation2d();
            Rotation2d currentAngle = RobotGyro.getRotation2d();

            double angleDiff = MathUtil.inputModulus(targetAngle.minus(currentAngle).plus(Rotation2d.k180deg).getDegrees(), -180, 180);
            rotSpeed = MathUtil.clamp(angleDiff * kPRotation, -0.5, 0.5);
        }

        // System.out.println("forward speed: " + ySpeed + ", x speed: " + xSpeed);
        // System.out.println("y: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftVerticalMovement()) + ", x: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftHorizontalMovement()));

        this.driveTrain.drive(ySpeedError, xSpeedError, rotSpeed, false);
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}