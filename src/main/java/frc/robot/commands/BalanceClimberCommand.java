package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Flags;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.staticsubsystems.RobotGyro;

/*
 * Idea: Other command will lift at a predetermined power for a predetermined amount
 * of time, and then this command will balance it, assuming that the robot has 
 * already climbed. (with timeout duration so it doesn't get stuck in a loop)
 * 
 * This command left as exercise to reader.
 */
public class BalanceClimberCommand extends Command {
    private final ClimberSubsystem climber;
    private Timer timer;
    private static double BALANCE_CLIMBER_COMMAND_DURATION = 3.0;
    private PIDController pidController;

    public BalanceClimberCommand(ClimberSubsystem climber) {
        this.climber = climber;
        this.timer = new Timer();
        
        // initialize PID controller here with found values

        addRequirements(climber);
    }


    @Override
    public void initialize() {
        this.timer.restart();
    }

    @Override
    public void execute() {
        // move based on whichever axis we found
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.climber.setRawSpeed(0.0);
    }

    public boolean weAreBalanced() {
        // whatever angle is less than some delta.
        return true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(BALANCE_CLIMBER_COMMAND_DURATION) || weAreBalanced();
    }
}
