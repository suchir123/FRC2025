package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.staticsubsystems.RobotGyro;

/*
 * Idea: Other command will lift at a predetermined power for a predetermined amount
 * of time, and then this command will balance it, assuming that the robot has
 * already climbed. (with timeout duration so it doesn't get stuck in a loop)
 *
 * This command left as exercise to reader.
 */
public class BalanceClimberCommand extends Command {
    private static final double MAX_ADJUSTMENT_SPEED = 0.02;
    private static final double BALANCE_CLIMBER_COMMAND_DURATION = 3.0;
    private final ClimberSubsystem climber;
    private final Timer timer;
    private final PIDController pidController;

    public BalanceClimberCommand(ClimberSubsystem climber) {
        this.climber = climber;
        this.timer = new Timer();

        // initialize PID controller here with found values
        this.pidController = new PIDController(0.01, 0, 0);

        addRequirements(climber);
    }


    @Override
    public void initialize() {
        this.timer.restart();
    }

    @Override
    public void execute() {
        // move based on whichever axis we found
        double correction = MathUtil.clamp(pidController.calculate(RobotGyro.getGyroAngleByAxis(ClimberSubsystem.ROBOT_TILT_AXIS), 0), -MAX_ADJUSTMENT_SPEED, MAX_ADJUSTMENT_SPEED);

        // climber.setRawSpeed(correction);
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
