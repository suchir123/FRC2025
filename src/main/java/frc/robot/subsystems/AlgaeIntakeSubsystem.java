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

public class AlgaeIntakeSubsystem extends SubsystemBase{ // This subsystem needing much more feature only has 1 neo currently

    private final SparkMax algaeIntakeMotor;

    public AlgaeIntakeSubsystem(){
        algaeIntakeMotor = new SparkMax(Constants.PortConstants.CAN.ALGAE_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();

        algaeMotorConfig
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .voltageCompensation(12);
//        algaeMotorConfig.closedLoop
//                .pidf(1, 0, 0, 0)
//                .outputRange(-0.6, 0.6);

        algaeIntakeMotor.configure(algaeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    public void setRawSpeed(double speed) {
        algaeIntakeMotor.set(speed);
    }
}
