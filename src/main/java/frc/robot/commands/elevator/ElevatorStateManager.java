package frc.robot.commands.elevator;

import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorStateManager {
    public static final ElevatorStateManager INSTANCE = new ElevatorStateManager(0, new Rotation2d(), ElevatorStateManager.CoralIntakeState.STOPPED, false);

    private final ElevatorState currentState;

    static class ElevatorState {
        double height;
        Rotation2d pivotAngle;
        CoralIntakeState coralIntakeState;
        boolean runAlgaeRemover;
    }

    private ElevatorStateManager(double height, Rotation2d pivotAngle, CoralIntakeState coralIntakeState, boolean runAlgaeRemover) {
        this.currentState = new ElevatorState();

        this.currentState.height = height;
        this.currentState.pivotAngle = pivotAngle;
        this.currentState.coralIntakeState = coralIntakeState;
        this.currentState.runAlgaeRemover = runAlgaeRemover;
    }

    public double getHeight() {
        return this.currentState.height;
    }

    public Rotation2d getPivotAngle() {
        return this.currentState.pivotAngle;
    }

    public CoralIntakeState getCoralIntakeState() {
        return this.currentState.coralIntakeState;
    }

    public boolean getRunAlgaeRemover() {
        return this.currentState.runAlgaeRemover;
    }

    public ElevatorStateManager setHeight(double height) {
        this.currentState.height = height;
        return this;
    }

    public ElevatorStateManager setPivotAngle(Rotation2d pivotAngle) {
        this.currentState.pivotAngle = pivotAngle;
        return this;
    }

    public ElevatorStateManager setCoralIntakeState(CoralIntakeState coralIntakeState) {
        this.currentState.coralIntakeState = coralIntakeState;
        return this;
    }

    public ElevatorStateManager setRunAlgaeRemover(boolean runAlgaeRemover) {
        this.currentState.runAlgaeRemover = runAlgaeRemover;
        return this;
    }

    public enum CoralIntakeState {
        STOPPED,
        INTAKE,
        OUTTAKE
    }
}