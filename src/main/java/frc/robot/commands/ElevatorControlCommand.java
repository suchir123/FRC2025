package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorControlCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final AbstractController controller;

    private final Trigger uB, leftB, lowB, rB;
    
    public ElevatorControlCommand(ElevatorSubsystem elevator, AbstractController c) {
        this.elevator = elevator;
        this.controller = c;

        uB = c.upperButton();
        leftB = c.leftButton();
        lowB = c.lowerButton();
        rB = c.rightButton();

        addRequirements(elevator);
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // System.out.println("sentir");
        uB.onTrue(new InstantCommand(() -> this.elevator.setTargetHeight(1)));
        leftB.onTrue(new InstantCommand(() -> {
            this.elevator.setTargetHeight(0.5);
            // System.out.println("up");
        }));
        lowB.onTrue(new InstantCommand(() -> {
            this.elevator.setTargetHeight(0);
            // System.out.println("down");
        }));
        rB.onTrue(new InstantCommand(() -> this.elevator.setTargetHeight(0.2424)));
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
