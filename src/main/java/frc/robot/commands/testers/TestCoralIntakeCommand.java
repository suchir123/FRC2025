package frc.robot.commands.testers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class TestCoralIntakeCommand extends Command {
    private final CoralIntakeSubsystem coralIntake;
    private final AbstractController joystick;

    public TestCoralIntakeCommand(CoralIntakeSubsystem coralIntake, AbstractController joystick) {
        this.coralIntake = coralIntake;
        this.joystick = joystick;

        addRequirements(coralIntake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        coralIntake.setRawSpeed(joystick.getRightVerticalMovement());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
