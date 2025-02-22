package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.elevator.ElevatorStateManager;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.staticsubsystems.RobotGyro;

public class WaitForElevatorToGetToSetpointCommand extends Command {
    public WaitForElevatorToGetToSetpointCommand() {
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ElevatorStateManager.INSTANCE.atSetpoint();
    }
}
