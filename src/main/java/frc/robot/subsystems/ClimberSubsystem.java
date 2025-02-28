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

public class ClimberSubsystem extends SubsystemBase {
    public static final ADIS16470_IMU.IMUAxis ROBOT_TILT_AXIS = IMUAxis.kYaw;
    private static final double UPPER_HARD_LIMIT = 0.822;
    // 0.8205 = climb start position
    // set hard stop to 0.825

    // private final ThroughboreEncoder throughboreEncoder;

    private final SparkMax climbMotor;
    private final RelativeEncoder climbMotorEncoder;
    private final AbsoluteEncoder climbMotorAbsoluteEncoder;

    private static final GenericPublisher climbMotorEncoderVelocityPublisher = NetworkTablesUtil.getPublisher("robot", "climbMotorVelocity", NetworkTableType.kDouble);
    private static final GenericPublisher climbMotorEncoderPositionPublisher = NetworkTablesUtil.getPublisher("robot", "climbMotorPosition", NetworkTableType.kDouble);

    private final SparkClosedLoopController pidController;

    public ClimberSubsystem() {
        climbMotor = new SparkMax(Constants.PortConstants.CAN.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        climbMotorEncoder = climbMotor.getEncoder();
        climbMotorAbsoluteEncoder = climbMotor.getAbsoluteEncoder();
        // throughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.DIO.CLIMBER_ABSOLUTE_ENCODER_ABS_PORT, 0, false);
        pidController = climbMotor.getClosedLoopController();

        SparkMaxConfig climbMotorConfig = new SparkMaxConfig();

        climbMotorConfig
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .voltageCompensation(12);
        climbMotorConfig.closedLoop
                .pidf(7, 0, 0, 0)
                .outputRange(-0.6, 0.6)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        climbMotor.configure(climbMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        // throughboreEncoder.name = "climber";
    }

    @Override
    public void periodic() {
        climbMotorEncoderVelocityPublisher.setDouble(climbMotorEncoder.getVelocity());
        climbMotorEncoderPositionPublisher.setDouble(climbMotorEncoder.getPosition());
        System.out.println("climbMotorAbsoluteEncoder.getPosition() is " + climbMotorAbsoluteEncoder.getPosition());
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
            climbMotor.set(speed);
        }
    }

    public Rotation2d getThroughboreEncoderDistance() {
        return new Rotation2d();// throughboreEncoder.getTotalDistance();
    }

    public Rotation2d getAbsolutePosition() {
        return new Rotation2d(); // this.throughboreEncoder.getAbsolutePosition();
    }
}
