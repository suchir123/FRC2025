package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.staticsubsystems.RobotGyro;

/*
 * Idea: This command will lift at a predetermined power for a predetermined amount
 * of time, and then another command will balance it, assuming that the robot has
 * already climbed.
 *
 * Other command left as exercise to reader.
 */
public class ClimbCommand extends Command {
    private static double CLIMB_COMMAND_DURATION = 3.0;
    private static double CLIMB_COMMAND_END_ANGLE = 60.0;
    private static double SPEED = 0.2;
    private final ClimberSubsystem climber;
    private Timer timer;
    private Rotation2d currentPosition;

    public ClimbCommand(ClimberSubsystem climber) {
        this.climber = climber;
        this.timer = new Timer();
        this.currentPosition = this.climber.getAbsolutePosition();

        addRequirements(climber);
    }


    @Override
    public void initialize() {
        this.timer.restart();
    }

    @Override
    public void execute() {
        double pitch = RobotGyro.getGyroAngleDegreesPitch();
        double roll = RobotGyro.getGyroAngleDegreesRoll();
        double yaw = RobotGyro.getGyroAngleDegreesYaw();
        Rotation2d position = this.climber.getAbsolutePosition();

        System.out.println(
                "pitch = " + pitch +
                        "\nroll = " + roll +
                        "\nyaw = " + yaw +
                        "\nposition = " + position +
                        "\ntime = " + timer.get());

        if (false) {
            this.climber.setRawSpeed(SPEED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.climber.setRawSpeed(0.0);
    }

    public boolean endAnglePassed() {
        // implementation depends on if the gyro is reversed or not,
        // and which direction it is.
        // TODO!
        return false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(CLIMB_COMMAND_DURATION) || endAnglePassed();
    }
}
