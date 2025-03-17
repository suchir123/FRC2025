package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Flags;

public class AlgaeGroundIntakeSubsystem extends SubsystemBase {
    private final SparkMax algaeIntakeMotor;
    private final Servo algaeLeftServo;
    private final Servo algaeRightServo;

    private boolean down = false;

    public AlgaeGroundIntakeSubsystem() {
        algaeIntakeMotor = new SparkMax(PortConstants.CAN.ALGAE_GROUND_INTAKE_MOTOR_ID, MotorType.kBrushless);
        // leaving these undefined seems better than adding them with bad values
        algaeLeftServo = new Servo(PortConstants.PWM.ALGAE_LEFT_SERVO_PORT);
        algaeRightServo = new Servo(PortConstants.PWM.ALGAE_RIGHT_SERVO_PORT);
        
        algaeLeftServo.setBoundsMicroseconds(2500, 0, 0, 0, 500);
        algaeRightServo.setBoundsMicroseconds(2500, 0, 0, 0, 500);

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
        // System.out.println("DOWN? " + this.down);
        if (Flags.AlgaeGroundIntake.ENABLED) {
            if (this.down) {
                this.flapToValue(0, 1);
            } else {
                this.flapToValue(1, 0);
            }
        }
    }

    public void setIntakeSpeed(double speed) {
        if(Flags.AlgaeGroundIntake.ENABLED) {
            algaeIntakeMotor.set(speed);
        }
    }

    public void toggleServosDown() {
        this.down = !this.down;
    }

    public boolean servosDown() {
        return this.down;
    }

    /**
     * @param left  Value [0,1]
     * @param right Value [0,1]
     */
    public void flapToValue(double left, double right) {
        // System.out.println("left: " + this.leftServo.getAngle() + ", right: " + this.rightServo.getAngle());
        if(Flags.AlgaeGroundIntake.ENABLED) {
            this.algaeLeftServo.set(left);
            this.algaeRightServo.set(right);
        }
    }
}
