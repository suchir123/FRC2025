package frc.robot.commands.autons;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorStateManager;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.Util;

public class ElevatorAutonManager {
    private final BooleanSupplier getAtSetpoint;
    private final BooleanSupplier getIsCoralInIntake;
    private final DriveTrainSubsystem driveTrain;

    public ElevatorAutonManager(ElevatorSubsystem elevator, CoralIntakeSubsystem coralIntake, DriveTrainSubsystem driveTrain)
    {
        this.getAtSetpoint = elevator::getAtSetpoint;
        this.getIsCoralInIntake = coralIntake::hasCoral;
        this.driveTrain = driveTrain;
    }

    public Command resetPositionToReef6Command() {
        Pose2d reef6Pose = new Pose2d(5.143, 2.875, Rotation2d.fromDegrees(120));
        return new InstantCommand(() -> driveTrain.setPose(reef6Pose));
    }

    public Command resetGyroCommand() {
        return new InstantCommand(() -> {
                    if (Util.onBlueTeam()) {
                        RobotGyro.resetGyroAngle();
                    } else {
                        RobotGyro.setGyroAngle(180);
                    }
                    this.driveTrain.setHeadingLockMode(false);
                });
    }

    public Command getPlaceCoralCommand() {
        return 
            // new WaitCommand(0.0)
            // .andThen(
                new InstantCommand(() -> {
                    System.out.println("PlaceCoralCommand InstantCommand ran!");
                    ElevatorStateManager.INSTANCE.cloneState()
                        .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.INTAKE_FORCE_02)
                        .setAsCurrent();
                }
            ).andThen(
                new WaitCommand(0.6)
            );
    }

    public Command getStopIntakeCommand() {
        return new InstantCommand(() -> {
            System.out.println("getCoralIntake STOPPING InstantCommand ran!");
            ElevatorStateManager.INSTANCE.cloneState()
                .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
                .setAsCurrent();
        });
    }
    
    public Command getCoralIntakeCommand() {
        return new InstantCommand(() -> {
            System.out.println("getCoralIntake InstantCommand ran!");
            ElevatorStateManager.INSTANCE.cloneState()
                .setHeight(0)
                .setPivotAngle(Rotation2d.fromRotations(0.14))
                .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.INTAKE_FORCE_035)
                .setRunAlgaeRemover(false)
                .setAsCurrent();
        }).andThen(
            new WaitUntilCommand(getIsCoralInIntake)
        ).andThen(
            getStopIntakeCommand()
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
            new WaitUntilCommand(getAtSetpoint)
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
            new WaitUntilCommand(getAtSetpoint)
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
            new WaitUntilCommand(getAtSetpoint)
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
            new WaitUntilCommand(getAtSetpoint)
        );
    }

    public Command getGoToL4Command() {
        return new InstantCommand(() -> {
            System.out.println("InstantCommand L4 ran!");
            ElevatorStateManager.INSTANCE.cloneState()
                .setHeight(1.02)
                .setPivotAngle(Rotation2d.fromRotations(0.42))
                .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
                .setRunAlgaeRemover(false)
                .setAsCurrent();
        }).andThen(
            new WaitUntilCommand(getAtSetpoint)
        );
    }
}
