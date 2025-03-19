package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Flags;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.Util;

public class ClimberSubsystem extends SubsystemBase {
    public static final ADIS16470_IMU.IMUAxis ROBOT_TILT_AXIS = IMUAxis.kYaw;
    private static final double UPPER_HARD_LIMIT = 0.82;
    // private final ThroughboreEncoder throughboreEncoder;

    private final SparkMax leftClimbMotor;
    private final SparkMax rightClimbMotor;
    
    private final AbsoluteEncoder climbMotorAbsoluteEncoder;
    
    private final RelativeEncoder leftClimbMotorEncoder;
    private final RelativeEncoder rightClimbMotorEncoder;
    
    private static final GenericPublisher lClimbMotorEncoderVelocityPublisher = NetworkTablesUtil.getPublisher("robot", "lClimbMotorVelocity", NetworkTableType.kDouble);
    private static final GenericPublisher lClimbMotorEncoderPositionPublisher = NetworkTablesUtil.getPublisher("robot", "lClimbMotorPosition", NetworkTableType.kDouble);
    
    private static final GenericPublisher rClimbMotorEncoderVelocityPublisher = NetworkTablesUtil.getPublisher("robot", "rClimbMotorVelocity", NetworkTableType.kDouble);
    private static final GenericPublisher rClimbMotorEncoderPositionPublisher = NetworkTablesUtil.getPublisher("robot", "rClimbMotorPosition", NetworkTableType.kDouble);
    
    
    private final SparkClosedLoopController pidController;

    public ClimberSubsystem() {
        leftClimbMotor = new SparkMax(Constants.PortConstants.CAN.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
        leftClimbMotorEncoder = leftClimbMotor.getEncoder();
        
        rightClimbMotor = new SparkMax(Constants.PortConstants.CAN.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
        rightClimbMotorEncoder = rightClimbMotor.getEncoder();
        
        climbMotorAbsoluteEncoder = leftClimbMotor.getAbsoluteEncoder();
        // throughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.DIO.CLIMBER_ABSOLUTE_ENCODER_ABS_PORT, 0, false);
        pidController = leftClimbMotor.getClosedLoopController();

        SparkMaxConfig leftClimbMotorConfig = new SparkMaxConfig();

        leftClimbMotorConfig
            .inverted(false)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .voltageCompensation(12);
        leftClimbMotorConfig.closedLoop
            .pidf(7, 0, 0, 0)
            .outputRange(-0.6, 0.6)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        
        SparkMaxConfig rightClimbMotorConfig = new SparkMaxConfig();
        
        rightClimbMotorConfig
            .follow(leftClimbMotor, true);
        
        Util.configureSparkMotor(leftClimbMotor, leftClimbMotorConfig);
        Util.configureSparkMotor(rightClimbMotor, rightClimbMotorConfig);
        // leftClimbMotor.configure(leftClimbMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        // rightClimbMotor.configure(rightClimbMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        // throughboreEncoder.name = "climber";
    }

    @Override
    public void periodic() {
        lClimbMotorEncoderVelocityPublisher.setDouble(leftClimbMotorEncoder.getVelocity());
        lClimbMotorEncoderPositionPublisher.setDouble(leftClimbMotorEncoder.getPosition());
        
        rClimbMotorEncoderVelocityPublisher.setDouble(rightClimbMotorEncoder.getVelocity());
        rClimbMotorEncoderPositionPublisher.setDouble(rightClimbMotorEncoder.getPosition());
        
        //System.out.println("climbMotorAbsoluteEncoder.getPosition() is " + climbMotorAbsoluteEncoder.getPosition());
        
        if(this.leftClimbMotor.get() > 0 && this.climbMotorAbsoluteEncoder.getPosition() >= UPPER_HARD_LIMIT - 0.01) {
            leftClimbMotor.set(0);
        }
        // throughboreEncoder.periodic();
    }

    public void setTargetRotationCount(double target) {
        if (Flags.Climber.ENABLED) {
            target = MathUtil.clamp(target, 0, UPPER_HARD_LIMIT);
            pidController.setReference(target, SparkBase.ControlType.kPosition);
        }
    }

    public void setRawSpeed(double speed) {
        if (Flags.Climber.ENABLED) {
            if(speed > 0 && this.climbMotorAbsoluteEncoder.getPosition() >= UPPER_HARD_LIMIT - 0.01) {
                return;
            }
            leftClimbMotor.set(speed);
        }
    }

    public Rotation2d getThroughboreEncoderDistance() {
        return new Rotation2d();// throughboreEncoder.getTotalDistance();
    }

    public Rotation2d getAbsolutePosition() {
        return new Rotation2d(); // this.throughboreEncoder.getAbsolutePosition();
    }
}
