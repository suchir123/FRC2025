package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.util.ThroughboreEncoder;

public class ClimberSubsystem extends SubsystemBase{
    private final ThroughboreEncoder throughboreEncoder;
    
    private final SparkMax climbMotor;

    private final SparkClosedLoopController pidController;

    public ClimberSubsystem(){
        climbMotor = new SparkMax(Constants.PortConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        throughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.CLIMBER_ABS_ENCODER_ID, 0, false);
        pidController = climbMotor.getClosedLoopController();

        
    }
}
