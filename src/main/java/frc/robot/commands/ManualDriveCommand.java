package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Flags;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.Util;

public class ManualDriveCommand extends Command {
    public static final double MAX_SPEED_METERS_PER_SEC = Flags.DriveTrain.LOWER_MAX_SPEED ? 1.5 : 3;
    private final DriveTrainSubsystem driveTrain;
    private final AbstractController joystick;
    // private final AprilTagHandler aprilTagHandler;
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(1);
    // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);
    // private final Trigger autoAimSubwoofer;
    // private final LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    // private boolean wasAutomaticallyDrivingLastFrame = false;

    public ManualDriveCommand(DriveTrainSubsystem driveTrain, AbstractController joystick) { //AprilTagHandler aprilTagHandler) {
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
        }
        return -1;
    }

    @Override
    public void execute() {
        // System.out.println("vert: " + this.joystick.getRightVerticalMovement() + ", hor: " + this.joystick.getRightHorizontalMovement());
        // this.driveTrain.drive(this.joystick.getVerticalMovement());
        double flip = flipFactor();
        double ySpeed = Util.squareKeepSign(this.ySpeedLimiter.calculate(-this.joystick.getLeftVerticalMovement() * flip)) * MAX_SPEED_METERS_PER_SEC;
        double xSpeed = Util.squareKeepSign(this.xSpeedLimiter.calculate(this.joystick.getLeftHorizontalMovement() * flip)) * MAX_SPEED_METERS_PER_SEC;
        // System.out.println("xSpeed = " + xSpeed);
        // System.out.println("ySpeed = " + ySpeed);

        double rotSpeed = -this.joystick.getRightHorizontalMovement() * 3.0;
        // System.out.println("rotSpeed = " + rotSpeed);

        this.driveTrain.drive(xSpeed, ySpeed, rotSpeed, true);
    }

    /**
     * Calculate the angle the gyroscope should be at in order to look at the speaker
     *
     * @return A Rotation2d representing the angle to the speaker. The gyroscope value should equal this value when the robot is facing the speaker.
     * //
     */
    // private Optional<Rotation2d> directionToSubwooferTarget() {
    //     int tagId = Util.getTargetTagId();
    //     Pose2d targetPose2d = Util.getTagPose(tagId).toPose2d();

    //     // i love Optional<T> :3
    //     return aprilTagHandler
    //             .averageAutoAimPose(tagId)
    //             .map((robotPose) -> // now we know where to aim, compare our current location with our target
    //                 Math.atan2(
    //                         targetPose2d.getY() - robotPose.getY(),
    //                         targetPose2d.getX() - robotPose.getX()
    //                 ) // trust me bro
    //             )
    //             // filter to decrease noise.
    //             .map(filter::calculate)
    //             .map(Rotation2d::new);
    // }

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
