package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ElevatorStateManager {
    /**
     * The global ElevatorStateManager instance. I feel like this doesn't make a whole lot of sense in the context of the program but it's easier to pass this around and I'm lowkey too lazy to make it non-static even though it's maybe a couple extra lines of code since only RobotContainer also needs it.
     */
    public static final ElevatorStateManager INSTANCE = new ElevatorStateManager(0, new Rotation2d(), ElevatorStateManager.CoralIntakeState.STOPPED, false);

    private ElevatorState currentState;
    private ElevatorState nextState;
    
    private ElevatorStateManager(double height, Rotation2d pivotAngle, CoralIntakeState coralIntakeState, boolean runAlgaeRemover) {
        this.currentState = new ElevatorState();
        this.nextState = null;

        this.currentState
                .setHeight(height)
                .setPivotAngle(pivotAngle)
                .setCoralIntakeState(coralIntakeState)
                .setRunAlgaeRemover(runAlgaeRemover);
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

    public ElevatorState getCurrentState() {
        return this.currentState;
    }

    public void setState(ElevatorState state) {
        if (state != null && state.verify()) {
            this.currentState = state;
        }
    }

    /**
     * Calls {@link ElevatorState#copy()} on the current state and returns the value.
     *
     * @return A copied object matching the current state.
     * @see ElevatorState#copy()
     * @see #getCurrentState()
     */
    public ElevatorState cloneState() {
        return this.getCurrentState().copy();
    }

    public ElevatorState getNextState() {
        return this.nextState;
    }

    /**
     * Sets the next queued state.
     *
     * @param state The state to queue.
     */
    public void setNextState(ElevatorState state) {
        this.nextState = state;
    }

    /**
     * Sets the current state to the next queued state.
     */
    public void pushNextState() {
        this.setState(this.getNextState());
        this.setNextState(null);
    }

    /**
     * The possible states for the coral intake.
     */
    public enum CoralIntakeState {
        STOPPED,
        INTAKE,
        OUTTAKE
    }

    /**
     * A holder for the actual target state of the elevator.
     */
    public class ElevatorState {
        double height;
        Rotation2d pivotAngle;
        CoralIntakeState coralIntakeState;
        boolean runAlgaeRemover;

        public double getHeight() {
            return height;
        }

        public ElevatorState setHeight(double height) {
            this.height = height;
            return this;
        }

        public Rotation2d getPivotAngle() {
            return pivotAngle;
        }

        public ElevatorState setPivotAngle(Rotation2d pivotAngle) {
            this.pivotAngle = pivotAngle;
            return this;
        }

        public CoralIntakeState getCoralIntakeState() {
            return coralIntakeState;
        }

        public ElevatorState setCoralIntakeState(CoralIntakeState coralIntakeState) {
            this.coralIntakeState = coralIntakeState;
            return this;
        }

        public boolean isRunAlgaeRemover() {
            return runAlgaeRemover;
        }

        public ElevatorState setRunAlgaeRemover(boolean runAlgaeRemover) {
            this.runAlgaeRemover = runAlgaeRemover;
            return this;
        }

        /**
         * @return A copy of the current state. Note: all the things in here are non-mutable and we treat them as such (making copies/reading final data values), but don't treat this as a proper {@link Cloneable} clone.
         */
        public ElevatorState copy() {
            ElevatorState copy = new ElevatorState();
            copy
                    .setHeight(this.height)
                    .setPivotAngle(this.pivotAngle)
                    .setCoralIntakeState(this.coralIntakeState)
                    .setRunAlgaeRemover(this.runAlgaeRemover);
            return copy;
        }

        /**
         * Primes this state object as the next state in queue to be applied.
         */
        public void primeAsNext() {
            ElevatorStateManager.this.setNextState(this); // i did not think using non-static inner classes would ever be useful, this is a first i love nested inner classes
        }

        /**
         * Sets this state object as the current target state.
         */
        public void setAsCurrent() {
            this.primeAsNext();
            ElevatorStateManager.this.pushNextState();

        }

        /**
         * @return If the elements of the state are not-null. NOTE: THIS DOES NOT ELIMINATE THE NEED FOR BOUNDS CHECKS. THIS IS LEFT TO OTHER IMPLEMENTATION CODE.
         */
        public boolean verify() {
            return pivotAngle != null && coralIntakeState != null;
        }
    }
}