package frc.robot.commands.autons;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.AprilTagUtil;
import frc.robot.util.Util;

// Need to tune speed and have different times!
public class FollowApriltagForwardCommand extends Command {
    public static final double MAX_SPEED_METERS_PER_SEC = DriveTrainSubsystem.MAX_SPEED_METERS_PER_SEC;
    public static final double MAX_ROT_SPEED_ANGULAR = 3;
    private final DriveTrainSubsystem driveTrain;
    private final Timer timer;
    private final double duration;
    // private boolean endNow;
    private boolean taperSpeed;

    // 0.6 is a made up constant - tune it!!!
    private static final double AUTO_SPEED = MAX_SPEED_METERS_PER_SEC * 0.6;

    // private final AprilTagHandler aprilTagHandler;
    // private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(1);
    // private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(1);
    // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);
    // private final Trigger autoAimSubwoofer;
    // private final LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    // private boolean wasAutomaticallyDrivingLastFrame = false;

    public FollowApriltagForwardCommand(DriveTrainSubsystem driveTrain, double duration, boolean taperSpeed) { //AprilTagHandler aprilTagHandler) {
        this.driveTrain = driveTrain;
        this.timer = new Timer();
        this.duration = duration;
        this.taperSpeed = taperSpeed;
        // this.endNow = false;
        // this.aprilTagHandler = aprilTagHandler;
        // this.autoAimSubwoofer = ControlHandler.get(joystick, ControllerConstants.AUTO_AIM_FOR_SHOOT);

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        this.timer.restart();
        // this.driveTrain.setPose(new Pose2d(2, 7, RobotGyro.getRotation2d()));
    }
    
    @Override
    public void execute() {
        if(Util.onBlueTeam()) {
            LimeLight.setLimeyPipeline(1);
        } else {
            LimeLight.setLimeyPipeline(0);
        }
        double flip = DriveTrainSubsystem.flipFactor();
        // System.out.println("vert: " + this.joystick.getRightVerticalMovement() + ", hor: " + this.joystick.getRightHorizontalMovement());
        // this.driveTrain.drive(this.joystick.getVerticalMovement());
        final double kPTranslation = 0.2;
        final double kPRotation = 0.1;
        //double flip = flipFactor();
        double pixelDiff = -LimeLight.getLimeyTX();
        int tagId = LimeLight.getLimeyTargetTag();
        // double ySpeedError = -Util.squareKeepSign(this.ySpeedLimiter.calculate(this.joystick.getLeftVerticalMovement() * flip)) * MAX_SPEED_METERS_PER_SEC;
        double xSpeedError = MathUtil.clamp(kPTranslation * pixelDiff, -0.3, 0.3);
        // System.out.println("xSpeed = " + xSpeed);
        // System.out.println("ySpeed = " + ySpeed);

        Optional<Pose3d> tagPose = AprilTagUtil.getTagPose(tagId);
        double rotSpeed = 0;
        // if(tagPose.isPresent()) {
        //     Rotation2d targetAngle = AprilTagUtil.getTagPose(tagId).orElseGet(Pose3d::new).getRotation().toRotation2d();
        //     Rotation2d currentAngle = RobotGyro.getRotation2d();

        //     double angleDiff = MathUtil.inputModulus(targetAngle.minus(currentAngle).plus(Rotation2d.k180deg).getDegrees(), -180, 180);
        //     // rotSpeed = MathUtil.clamp(angleDiff * kPRotation, -0.5, 0.5);
        // }

        // System.out.println("forward speed: " + ySpeed + ", x speed: " + xSpeed);
        // System.out.println("y: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftVerticalMovement()) + ", x: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftHorizontalMovement()));

        double forwardSpeed;
        if (taperSpeed) {
            double timeLeft = this.duration - this.timer.get();
            double speedModifier = timeLeft / this.duration;
            forwardSpeed = speedModifier * AUTO_SPEED;
            // System.out.println(timeLeft + "<" + this.duration * 0.25 + " : " + (timeLeft < this.duration * 0.25));
            // if (timeLeft < this.duration * 0.25) {
            //     endNow = true;
            // }
        } else {
            // uhhhh go slower i guess
            forwardSpeed = AUTO_SPEED / 2;
        }

        System.out.println("xSpeedError=" + xSpeedError);
        this.driveTrain.drive(forwardSpeed, xSpeedError, rotSpeed, false);
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.driveTrain.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(this.duration * 0.75);
    }
}