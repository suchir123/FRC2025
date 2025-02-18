package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Flags;
import frc.robot.util.ThroughboreEncoder;

public class ClimberSubsystem extends SubsystemBase {
    public static final ADIS16470_IMU.IMUAxis ROBOT_TILT_AXIS = IMUAxis.kYaw;

    // private final ThroughboreEncoder throughboreEncoder;

    private final SparkMax climbMotor;

    private final SparkClosedLoopController pidController;

    public ClimberSubsystem() {
        climbMotor = new SparkMax(Constants.PortConstants.CAN.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        // throughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.DIO.CLIMBER_ABSOLUTE_ENCODER_ABS_PORT, 0, false);
        pidController = climbMotor.getClosedLoopController();

        SparkMaxConfig climbMotorConfig = new SparkMaxConfig();

        climbMotorConfig
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .voltageCompensation(12);
        climbMotorConfig.closedLoop
                .pidf(1, 0, 0, 0)
                .outputRange(-0.6, 0.6);

        climbMotor.configure(climbMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        // throughboreEncoder.name = "climber";
    }

    @Override
    public void periodic() {
        // throughboreEncoder.periodic();
    }

    public void setTarget(double target) {
        if (Flags.Climber.ENABLED) {
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
