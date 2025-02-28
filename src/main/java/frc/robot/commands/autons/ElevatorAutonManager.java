package frc.robot.commands.autons;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorStateManager;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorAutonManager {
    private final BooleanSupplier getAtSetpoint;

    public ElevatorAutonManager(ElevatorSubsystem elevator)
    {
        this.getAtSetpoint = elevator::getAtSetpoint;
    }

    public Command getPlaceCoralCommand() {
        return new InstantCommand(() -> {
            System.out.println("PlaceCoralCommand InstantCommand ran!");
            ElevatorStateManager.INSTANCE.cloneState()
                .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.INTAKE)
                .setAsCurrent();
        }).andThen(
            new WaitCommand(1.0)
        );
    }
    
    public Command getGoToIntakeStateCommand() {
        return new InstantCommand(() -> {
            System.out.println("getGoToIntakeState InstantCommand ran!");
            ElevatorStateManager.INSTANCE.cloneState()
                .setHeight(0)
                .setPivotAngle(Rotation2d.fromRotations(0.14))
                .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
                .setRunAlgaeRemover(false)
                .setAsCurrent();
        }).andThen(
            new WaitForElevatorToGetToSetpointCommand(getAtSetpoint)
        );
    }

    public Command getGoToL1Command() {
        return new InstantCommand(() -> ElevatorStateManager.INSTANCE.cloneState()
            .setHeight(0)
            .setPivotAngle(Rotation2d.fromRotations(0.03))
            .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
            .setRunAlgaeRemover(false)
            .setAsCurrent()
        ).andThen(
            new WaitForElevatorToGetToSetpointCommand(getAtSetpoint)
        );
    }

    public Command getGoToL2Command() {
        return new InstantCommand(() -> ElevatorStateManager.INSTANCE.cloneState()
            .setHeight(0)
            .setPivotAngle(Rotation2d.fromRotations(0.46))
            .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
            .setRunAlgaeRemover(false)
            .setAsCurrent()
        ).andThen(
            new WaitForElevatorToGetToSetpointCommand(getAtSetpoint)
        );
    }

    public Command getGoToL3Command() {
        return new InstantCommand(() -> ElevatorStateManager.INSTANCE.cloneState()
            .setHeight(0.35)
            .setPivotAngle(Rotation2d.fromRotations(0.46))
            .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
            .setRunAlgaeRemover(false)
            .setAsCurrent()
        ).andThen(
            new WaitForElevatorToGetToSetpointCommand(getAtSetpoint)
        );
    }

    public Command getGoToL4Command() {
        return new InstantCommand(() -> {
            System.out.println("InstantCommand L4 ran!");
            ElevatorStateManager.INSTANCE.cloneState()
                .setHeight(0.99)
                .setPivotAngle(Rotation2d.fromRotations(0.44))
                .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
                .setRunAlgaeRemover(false)
                .setAsCurrent();
        }).andThen(
            new WaitForElevatorToGetToSetpointCommand(getAtSetpoint)
        ).andThen(
            new WaitCommand(0.5)
        );
    }
}
