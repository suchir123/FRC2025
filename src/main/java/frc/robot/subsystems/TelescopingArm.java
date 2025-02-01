package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Queue;

import org.ejml.simple.UnsupportedOperation;

import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ThroughboreEncoder;

public class TelescopingArm extends SubsystemBase {
    private final SparkMax leadingMotor;
    private final SparkMax followingMotor;

    private final RelativeEncoder leadingMotorEncoder;
    private final RelativeEncoder followingMotorEncoder;

    private final ThroughboreEncoder leadingThroughboreEncoder;
    private final ThroughboreEncoder followingThroughboreEncoder;
    private final PIDController followingPIDController;

    private final Rotation2d maximumDistance = Rotation2d.fromRotations(10.0);
    private final double baseSpeed = 0.1;
    // Because of mechanical problems, our left climbing shaft moves up more slowly, so it needs a boost.
    private double followingMotorSpeedBoost = 1.2;

    public TelescopingArm() {
        if (this.portsAreNotSet()) {
            throw new UnsupportedOperation("You didn't set the ports for TelescopingArm.");
        }

        int leadingMotorPort = Constants.PortConstants.CLIMBER_RIGHT_MOTOR_ID;
        int followingMotorPort = Constants.PortConstants.CLIMBER_LEFT_MOTOR_ID;
        leadingMotor = new SparkMax(leadingMotorPort, MotorType.kBrushless);
        followingMotor = new SparkMax(followingMotorPort, MotorType.kBrushless);
        leadingMotorEncoder = leadingMotor.getEncoder();
        followingMotorEncoder = followingMotor.getEncoder();

        leadingThroughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.RIGHT_CLIMB_ABS_ENCODER_ID, 
                                                           Constants.PortConstants.RIGHT_CLIMB_A_ENCODER_ID, 
                                                           Constants.PortConstants.RIGHT_CLIMB_B_ENCODER_ID);
        followingThroughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.LEFT_CLIMB_ABS_ENCODER_ID, 
                                                             Constants.PortConstants.LEFT_CLIMB_A_ENCODER_ID, 
                                                             Constants.PortConstants.LEFT_CLIMB_B_ENCODER_ID);
        
        // TODO: tune
        followingPIDController = new PIDController(0.055, 0, 0.03);
    }

    /*
     * Input: -1.0 <= leadingSpeed <= 1.0, -1.0 <= followingSpeed <= 1.0
     * 
     * Speeds scaled & clamped between -0.15 and 0.15, then sent to motors.
     */
    public void ascendSimple(double leadingSpeed, double followingSpeed) {
        final double MAX_SPEED = 0.15;
        leadingSpeed = leadingSpeed * MAX_SPEED;
        followingSpeed = followingSpeed * MAX_SPEED;

        leadingSpeed   = MathUtil.clamp(leadingSpeed,   -MAX_SPEED, MAX_SPEED);
        followingSpeed = MathUtil.clamp(followingSpeed, -MAX_SPEED, MAX_SPEED);

        leadingMotor.set(leadingSpeed);
        followingMotor.set(followingSpeed);
    }

    public Rotation2d getLeadingThroughboreEncoderDistance() {
        return this.leadingThroughboreEncoder.getAbsoluteDistance();
    }

    public Rotation2d getFollowingThroughboreEncoderDistance() {
        return this.followingThroughboreEncoder.getAbsoluteDistance();
    }

    public void ascendPID(double leadingSpeed) {

    }

    // Silly method, remove after we set the ports.
    private boolean portsAreNotSet() {
        if (Constants.PortConstants.RIGHT_CLIMB_MOTOR_ID       == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        if (Constants.PortConstants.RIGHT_CLIMB_ABS_ENCODER_ID == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        if (Constants.PortConstants.RIGHT_CLIMB_A_ENCODER_ID   == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        if (Constants.PortConstants.RIGHT_CLIMB_B_ENCODER_ID   == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        if (Constants.PortConstants.LEFT_CLIMB_MOTOR_ID        == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        if (Constants.PortConstants.LEFT_CLIMB_ABS_ENCODER_ID  == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        if (Constants.PortConstants.LEFT_CLIMB_A_ENCODER_ID    == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        if (Constants.PortConstants.LEFT_CLIMB_B_ENCODER_ID    == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        return false;
    }
}
