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
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ThroughboreEncoder;

public class TelescopingArm extends SubsystemBase {
    private final SparkMax rightMotor;
    private final SparkMax leftMotor;

    private final RelativeEncoder rightMotorEncoder;
    private final RelativeEncoder leftMotorEncoder;

    private final ThroughboreEncoder rightThroughboreEncoder;
    private final ThroughboreEncoder leftThroughboreEncoder;
    private final PIDController rightPIDController;
    private final PIDController leftPIDController;

    private static final double maximumDistanceMeters = 2.0;
    private static final double metersAscendedPerAbsoluteEncoderRotation = 0.1; // TODO: Figure out this constant.

    // Guaranteed not to send values 
    public static final double MAX_SPEED = 0.15;

    public TelescopingArm() {
        if (this.portsAreNotSet()) {
            throw new UnsupportedOperation("You didn't set the ports for TelescopingArm.");
        }

        int leadingMotorPort = Constants.PortConstants.CLIMBER_RIGHT_MOTOR_ID;
        int followingMotorPort = Constants.PortConstants.CLIMBER_LEFT_MOTOR_ID;
        rightMotor = new SparkMax(leadingMotorPort, MotorType.kBrushless);
        leftMotor = new SparkMax(followingMotorPort, MotorType.kBrushless);
        rightMotorEncoder = rightMotor.getEncoder();
        leftMotorEncoder = leftMotor.getEncoder();

        rightThroughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.RIGHT_CLIMB_ABS_ENCODER_ID, 0.0,false);
        leftThroughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.LEFT_CLIMB_ABS_ENCODER_ID, 0.0,false);
        
        // TODO: tune
        rightPIDController = new PIDController(0.055, 0, 0.03);
        leftPIDController  = new PIDController(0.055, 0, 0.03);
    }

    /*
     * inputs: heightMeters: the height of the setpoint in meters.
     */
    public void setHeight(double heightMeters) {
        
    }

    /*
     * Input: -1.0 <= leadingSpeed <= 1.0, -1.0 <= followingSpeed <= 1.0
     * 
     * Speeds scaled & clamped between -0.15 and 0.15, then sent to motors.
     */
    public void ascendSimple(double leftSpeed, double rightSpeed) {
        Pair<Double, Double> speeds = new Pair<Double, Double>(leftSpeed, rightSpeed);
        speeds = scaleSpeeds(speeds, 1.0, MAX_SPEED);
        speeds = clampSpeeds(speeds, MAX_SPEED);
        setRawSpeeds(speeds);
    }

    public void ascendWithLeftBoost(double speed, double leftBoost) {
        Pair<Double, Double> speeds = new Pair<Double, Double>(speed * leftBoost, speed);
        speeds = scaleSpeeds(speeds, 1.0, MAX_SPEED);
        speeds = desaturateSpeeds(speeds, MAX_SPEED);
        setRawSpeeds(speeds);
    }

    private static Pair<Double, Double> clampSpeeds(Pair<Double, Double> speeds, double maxSpeed) {
        double rightSpeed = MathUtil.clamp(speeds.getFirst(),  -maxSpeed, maxSpeed);
        double leftSpeed  = MathUtil.clamp(speeds.getSecond(), -maxSpeed, rightSpeed);
        return new Pair<Double, Double>(rightSpeed, leftSpeed);
    }

    private static Pair<Double, Double> scaleSpeeds(Pair<Double, Double> speeds, double initialMaxSpeed, double newMaxSpeed) {
        double scaleFactor = newMaxSpeed / initialMaxSpeed;
        return new Pair<Double, Double>(speeds.getFirst() * scaleFactor, speeds.getSecond() * scaleFactor);
    }

    private static Pair<Double, Double> desaturateSpeeds(Pair<Double, Double> speeds, double maxSpeed) {
        double leftSpeed = speeds.getFirst();
        double rightSpeed = speeds.getSecond();
        double largerSpeed = Math.max(leftSpeed, rightSpeed);
        if (largerSpeed >= maxSpeed) {
            double scaleFactor = maxSpeed / largerSpeed;
            leftSpeed *= scaleFactor;
            rightSpeed *= scaleFactor;
        }
        return new Pair<Double, Double>(leftSpeed, rightSpeed);
    }

    private void setRawSpeeds(double leftSpeed, double rightSpeed) {
        this.leftMotor.set(leftSpeed);
        this.rightMotor.set(rightSpeed);
    }

    private void setRawSpeeds(Pair<Double, Double> speeds) {
        this.leftMotor.set(speeds.getFirst());
        this.rightMotor.set(speeds.getSecond());
    }

    public Rotation2d getRightThroughboreEncoderDistance() {
        return this.rightThroughboreEncoder.getAbsoluteDistance();
    }

    public Rotation2d getLeftThroughboreEncoderDistance() {
        return this.leftThroughboreEncoder.getAbsoluteDistance();
    }

    // Silly method, remove after we set the ports.
    private boolean portsAreNotSet() {
        if (Constants.PortConstants.RIGHT_CLIMB_MOTOR_ID       == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        if (Constants.PortConstants.LEFT_CLIMB_MOTOR_ID        == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        if (Constants.PortConstants.RIGHT_CLIMB_ABS_ENCODER_ID == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        if (Constants.PortConstants.LEFT_CLIMB_ABS_ENCODER_ID  == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        // if (Constants.PortConstants.RIGHT_CLIMB_A_ENCODER_ID   == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        // if (Constants.PortConstants.RIGHT_CLIMB_B_ENCODER_ID   == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        // if (Constants.PortConstants.LEFT_CLIMB_A_ENCODER_ID    == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        // if (Constants.PortConstants.LEFT_CLIMB_B_ENCODER_ID    == Constants.SOME_MADE_UP_WRONG_PORT) return true;
        return false;
    }
}
