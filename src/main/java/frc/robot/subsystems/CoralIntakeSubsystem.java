package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Flags;

public class CoralIntakeSubsystem extends SubsystemBase {
    private final SparkMax coralPivotMotor;
    private final AbsoluteEncoder coralPivotAbsoluteEncoder;

    private final SparkClosedLoopController pidController;

    public CoralIntakeSubsystem() {
        coralPivotMotor = new SparkMax(Constants.PortConstants.CAN.CORAL_PIVOT_MOTOR_ID, MotorType.kBrushless);
        coralPivotAbsoluteEncoder = coralPivotMotor.getAbsoluteEncoder();
        // throughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.DIO.CORAL_ABSOLUTE_ENCODER_ABS_ID, 0, false);
        pidController = coralPivotMotor.getClosedLoopController();

        SparkMaxConfig coralPivotMotorConfig = new SparkMaxConfig();

        coralPivotMotorConfig
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .voltageCompensation(12);
        coralPivotMotorConfig.closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                .pidf(1, 0, 0, 0)
                .outputRange(-0.6, 0.6);
        coralPivotMotorConfig.absoluteEncoder
                .setSparkMaxDataPortConfig()
                .zeroOffset(0);

        coralPivotMotor.configure(coralPivotMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        // throughboreEncoder.name = "climber";
    }

    @Override
    public void periodic() {
        // throughboreEncoder.periodic();
    }

    public void setPivotTargetAngle(double target) {
        if (Flags.CoralIntake.ENABLED) {
            pidController.setReference(target, SparkBase.ControlType.kPosition);
        }
    }

    public void setRawSpeed(double speed) {
        if (Flags.CoralIntake.ENABLED) {
            coralPivotMotor.set(speed);
        }
    }

    public Rotation2d getThroughboreEncoderDistance() {
        return new Rotation2d();// throughboreEncoder.getTotalDistance();
    }
}
