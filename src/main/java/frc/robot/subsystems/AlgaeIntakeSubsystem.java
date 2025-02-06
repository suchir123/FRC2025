package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants;
import frc.robot.util.ThroughboreEncoder;

public class AlgaeIntakeSubsystem extends SubsystemBase{
    private final ThroughboreEncoder throughboreEncoder;

    private final SparkMax algaeIntakeMotor;

    private final SparkClosedLoopController pidController;

    public AlgaeIntakeSubsystem(){
        algaeIntakeMotor = new SparkMax(Constants.PortConstants.ALGAE_MOTOR_ID, MotorType.kBrushless);
        throughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.ALGAE_ABS_ENCODER_ID, 0, false);
        pidController = algaeIntakeMotor.getClosedLoopController();

        SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();

        algaeMotorConfig
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .voltageCompensation(12);
        algaeMotorConfig.closedLoop
                .pidf(1, 0, 0, 0)
                .outputRange(-0.6, 0.6);

        algaeIntakeMotor.configure(algaeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        throughboreEncoder.name = "climber";
    }

    @Override
    public void periodic() {
        throughboreEncoder.periodic();
    }

    public void setTarget(double target) {
        pidController.setReference(target, SparkBase.ControlType.kPosition);
    }

    public void setRawSpeed(double speed) {
        algaeIntakeMotor.set(speed);
    }

    public Rotation2d getThroughboreEncoderDistance() {
        return throughboreEncoder.getTotalDistance();
    }
}
