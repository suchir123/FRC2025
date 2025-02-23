package frc.robot.commands.autons;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitForElevatorToGetToSetpointCommand extends Command {
    private final BooleanSupplier getAtSetpoint;

    public WaitForElevatorToGetToSetpointCommand(BooleanSupplier getAtSetpoint) {
        this.getAtSetpoint = getAtSetpoint;
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
        return getAtSetpoint.getAsBoolean();
    }
}
