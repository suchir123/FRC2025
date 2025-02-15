package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortConstants;

public class AlgaeGroundIntakeSubsystem extends SubsystemBase {
    private final SparkMax algaeIntakeMotor;
    private final Servo algaeLeftServo;
    private final Servo algaeRightServo;

    public AlgaeGroundIntakeSubsystem() {
        algaeIntakeMotor = new SparkMax(Constants.PortConstants.CAN.ALGAE_REMOVER_MOTOR_ID, MotorType.kBrushless);
        // leaving these undefined seems better than adding them with bad values
        algaeLeftServo = new Servo(PortConstants.PWM.ALGAE_LEFT_SERVO_PORT);
        algaeRightServo = new Servo(PortConstants.PWM.ALGAE_RIGHT_SERVO_PORT);

        SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();

        algaeMotorConfig
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .smartCurrentLimit(20)
                .voltageCompensation(12);

        algaeIntakeMotor.configure(algaeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    public void setRawSpeed(double speed) {
        algaeIntakeMotor.set(speed);
    }

    public void flapToAngle(double degreesL, double degreesR) {
        // System.out.println("left: " + this.leftServo.getAngle() + ", right: " + this.rightServo.getAngle());
        this.algaeLeftServo.setAngle(degreesL);
        this.algaeRightServo.setAngle(degreesR);
    }
}
