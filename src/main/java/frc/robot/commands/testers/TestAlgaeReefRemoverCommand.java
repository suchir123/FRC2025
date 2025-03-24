package frc.robot.commands.testers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.AlgaeReefRemoverSubsystem;

public class TestAlgaeReefRemoverCommand extends Command {
    private final AlgaeReefRemoverSubsystem algaeReefRemover;
    private final AbstractController joystick;

    private final Trigger r;
    public TestAlgaeReefRemoverCommand(AlgaeReefRemoverSubsystem algaeReefRemover, AbstractController joystick) {
        this.algaeReefRemover = algaeReefRemover;
        this.joystick = joystick;

        r = joystick.rightButton();

        addRequirements(algaeReefRemover);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(r.getAsBoolean()) {
            algaeReefRemover.setIntakeSpeed(0.3);
        } else {
            algaeReefRemover.setIntakeSpeed(0);
        }
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
