package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.AlgaeGroundIntakeSubsystem;
import frc.robot.util.ControlHandler;

public class AlgaeGroundIntakeCommand extends Command {
    private final AlgaeGroundIntakeSubsystem algaeGroundIntake;
    private final AbstractController primaryController;
    private final AbstractController secondaryController;
    private final Trigger toggleAlgaeHeight;
    private final Trigger toggleCoralHeight;
    private final Trigger intakeButton;
    private final Trigger outtakeButton;
    private boolean atAlgaeHeight = false;
    private boolean atCoralHeight = false;

    public AlgaeGroundIntakeCommand(AlgaeGroundIntakeSubsystem algaeGroundIntake, AbstractController primaryController, AbstractController secondaryController) {
        this.algaeGroundIntake = algaeGroundIntake;
        this.primaryController = primaryController;
        this.secondaryController = secondaryController;

        this.toggleAlgaeHeight = ControlHandler.get(this.secondaryController, Constants.OperatorConstants.SecondaryControllerConstants.GROUND_INTAKE_ALGAE_TOGGLE);
        this.toggleCoralHeight = ControlHandler.get(this.secondaryController, Constants.OperatorConstants.SecondaryControllerConstants.GROUND_INTAKE_CORAL_TOGGLE);
        this.intakeButton = ControlHandler.get(this.primaryController, Constants.OperatorConstants.PrimaryControllerConstants.ALGAE_GROUND_INTAKE);
        this.outtakeButton = ControlHandler.get(this.primaryController, Constants.OperatorConstants.PrimaryControllerConstants.ALGAE_GROUND_OUTTAKE);

        addRequirements(algaeGroundIntake);
    }


    @Override
    public void initialize() {
        this.toggleAlgaeHeight.onTrue(new InstantCommand(() -> {
            if(!atAlgaeHeight) {
                this.algaeGroundIntake.flapToValue(0.4, 0.6);
            } else {
                this.algaeGroundIntake.flapToValue(0.9, 0.1);
            }
            atAlgaeHeight = !atAlgaeHeight;
            atCoralHeight = false;
        }));
        
        this.toggleCoralHeight.onTrue(new InstantCommand(() -> {
            if(!atCoralHeight) {
                this.algaeGroundIntake.flapToValue(0, 1);
            } else {
                this.algaeGroundIntake.flapToValue(0.9, 0.1);
            }
            atCoralHeight = !atCoralHeight;
            atAlgaeHeight = false;
        }));
    }

    @Override
    public void execute() {
        if (this.intakeButton.getAsBoolean()) {
            this.algaeGroundIntake.setIntakeSpeed(0.5);
        } else if (this.outtakeButton.getAsBoolean()) {
            this.algaeGroundIntake.setIntakeSpeed(-0.5);
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
