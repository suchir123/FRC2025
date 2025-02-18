package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.AlgaeGroundIntakeSubsystem;
import frc.robot.util.ControlHandler;

/*
 * Idea: Other command will lift at a predetermined power for a predetermined amount
 * of time, and then this command will balance it, assuming that the robot has
 * already climbed. (with timeout duration so it doesn't get stuck in a loop)
 *
 * This command left as exercise to reader.
 */
public class AlgaeGroundIntakeCommand extends Command {
    private final AlgaeGroundIntakeSubsystem algaeGroundIntake;
    private final AbstractController primaryController;
    private final AbstractController secondaryController;

    private boolean wasToggleServosPressed = false;
    private final Trigger toggleServosButton;
    private final Trigger intakeButton;
    private final Trigger outtakeButton;

    public AlgaeGroundIntakeCommand(AlgaeGroundIntakeSubsystem algaeGroundIntake, AbstractController primaryController, AbstractController secondaryController) {
        this.algaeGroundIntake = algaeGroundIntake;
        this.primaryController = primaryController;
        this.secondaryController = secondaryController;

        this.toggleServosButton = ControlHandler.get(this.secondaryController, Constants.OperatorConstants.SecondaryControllerConstants.TOGGLE_ALGAE_GROUND_INTAKE_HEIGHT);
        this.intakeButton = ControlHandler.get(this.primaryController, Constants.OperatorConstants.PrimaryControllerConstants.ALGAE_GROUND_INTAKE);
        this.outtakeButton = ControlHandler.get(this.primaryController, Constants.OperatorConstants.PrimaryControllerConstants.ALGAE_GROUND_OUTTAKE);

        addRequirements(algaeGroundIntake);
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!wasToggleServosPressed && this.toggleServosButton.getAsBoolean()) {
            wasToggleServosPressed = true;
            this.algaeGroundIntake.toggleServosDown();
        } else if (!this.toggleServosButton.getAsBoolean()) {
            wasToggleServosPressed = false;
        }

        if (this.intakeButton.getAsBoolean()) {
            this.algaeGroundIntake.setIntakeSpeed(0.35);
        } else if (this.outtakeButton.getAsBoolean()) {
            this.algaeGroundIntake.setIntakeSpeed(-0.35);
        } else {
            this.algaeGroundIntake.setIntakeSpeed(0);
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
