package frc.robot.commands.drive;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.AprilTagUtil;
import frc.robot.util.Util;

public class ReefAprilTagCenterCommand extends Command {
    public static final double MAX_SPEED_METERS_PER_SEC = DriveTrainSubsystem.MAX_SPEED_METERS_PER_SEC;
    public static final double MAX_ROT_SPEED_ANGULAR = 3;
    private final DriveTrainSubsystem driveTrain;
    private final AbstractController joystick;
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(1);

    public ReefAprilTagCenterCommand(DriveTrainSubsystem driveTrain, AbstractController joystick) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(Util.onBlueTeam()) {
            // LimeLight.setLimeyPipeline(1);
        } else {
            // LimeLight.setLimeyPipeline(2);
        }
        
        final double kPTranslation = 0.25;
        final double kPRotation = 0.1;
        //double flip = flipFactor();
        double pixelDiff = -LimeLight.getLimeyTX();
        if(Math.abs(pixelDiff) < 0.5) pixelDiff = 0;
        int tagId = LimeLight.getLimeyTargetTag();
        double ySpeedError = Util.squareKeepSign(this.ySpeedLimiter.calculate(this.joystick.getLeftVerticalMovement())) * MAX_SPEED_METERS_PER_SEC * 0.25;
        double xSpeedError = MathUtil.clamp(kPTranslation * pixelDiff, -0.069, 0.069);

        Optional<Pose3d> tagPose = AprilTagUtil.getTagPose(tagId);
        double rotSpeed = -this.joystick.getRightHorizontalMovement() * MAX_ROT_SPEED_ANGULAR;
        if(tagPose.isPresent()) {
            Rotation2d targetAngle = AprilTagUtil.getTagPose(tagId).orElseGet(Pose3d::new).getRotation().toRotation2d();
            Rotation2d currentAngle = RobotGyro.getRotation2d();

            double angleDiff = MathUtil.inputModulus(-targetAngle.minus(currentAngle).getDegrees(), -180, 180);
            if(Math.abs(angleDiff) < 2) angleDiff = 0;
            rotSpeed = MathUtil.clamp(angleDiff * kPRotation, -0.1, 0.1);
        }

        // System.out.println("forward speed: " + ySpeed + ", x speed: " + xSpeed);
        // System.out.println("y: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftVerticalMovement()) + ", x: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftHorizontalMovement()));

        System.out.println("driving at " + ySpeedError + " into the reef, correcting s2s at " + xSpeedError + ", correcting angle at " + rotSpeed);
        // NOTE: ROBOT RELATIVE DRIVE.
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