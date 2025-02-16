package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeReefRemoverSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorControlCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final CoralIntakeSubsystem coralIntake;
    private final AlgaeReefRemoverSubsystem algaeRemover;
    private final ElevatorState targetState;

    public ElevatorControlCommand(ElevatorSubsystem elevator, CoralIntakeSubsystem coralIntake, AlgaeReefRemoverSubsystem algaeRemover, ElevatorState targetState) {
        this.elevator = elevator;
        this.coralIntake = coralIntake;
        this.algaeRemover = algaeRemover;

        this.targetState = targetState;

        addRequirements(elevator, coralIntake, algaeRemover);
    }

    @Override
    public void initialize() {
        if(!this.targetState.maintain()) {
            this.elevator.setTargetHeight(this.targetState.height());
            this.coralIntake.setPivotTargetAngle(this.targetState.pivotAngle());
        }

        if (this.targetState.runAlgaeRemover()) {
            this.algaeRemover.setIntakeSpeed(0.2);
        }

        switch (this.targetState.coralIntakeState()) {
            case STOPPED:
                this.coralIntake.setIntakeSpeed(0);
                break;
            case INTAKE:
                this.coralIntake.setIntakeSpeed(0.6);
                break;
            case OUTTAKE:
                this.coralIntake.setIntakeSpeed(-0.6);
                break;
            default:
                System.out.println("ElevatorControlCommand.ElevatorState has invalid entries that aren't accounted for");
        }
    }

    @Override
    public void execute() {
        // System.out.println("JIOJFOIJSODFJFIOSJHFISHI:FJHEIL:HJF");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.algaeRemover.setIntakeSpeed(0);
        this.coralIntake.setIntakeSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public enum CoralIntakeState {
        STOPPED,
        INTAKE,
        OUTTAKE
    }

    public static class ElevatorState {
        private final double height;
        private final Rotation2d pivotAngle;
        private final boolean maintain;
        private final CoralIntakeState coralIntakeState;
        private final boolean runAlgaeRemover;

        private ElevatorState(double height, Rotation2d pivotAngle, boolean maintain, CoralIntakeState coralIntakeState, boolean runAlgaeRemover) {
            this.height = height;
            this.pivotAngle = pivotAngle;
            this.maintain = maintain;
            this.coralIntakeState = coralIntakeState;
            this.runAlgaeRemover = runAlgaeRemover;
        }
        
        public ElevatorState(double height, Rotation2d pivotAngle, CoralIntakeState coralIntakeState, boolean runAlgaeRemover) {
            this(height, pivotAngle, false, coralIntakeState, runAlgaeRemover);
        }

        public ElevatorState(CoralIntakeState coralIntakeState, boolean runAlgaeRemover) {
            this(-1, null, true, coralIntakeState, runAlgaeRemover);
        }

        public double height() {
            return this.height;
        }

        public Rotation2d pivotAngle() {
            return this.pivotAngle;
        }

        public boolean maintain() {
            return this.maintain;
        }

        public CoralIntakeState coralIntakeState() {
            return this.coralIntakeState;
        }

        public boolean runAlgaeRemover() {
            return this.runAlgaeRemover;
        }
    }
}

// at 0 alga is about 0.79 m above floor
// level 1 0.790 meter
// level 2 1.194 meter
// level 3 1.809 meter (also dont forget is like vertical pipe)