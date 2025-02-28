package frc.robot.commands.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeReefRemoverSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorControlCommand extends Command {
    private static final double BACK_LIMIT_WHEN_DOWN = 0.03;
    private final ElevatorSubsystem elevator;
    private final CoralIntakeSubsystem coralIntake;
    private final AlgaeReefRemoverSubsystem algaeRemover;
    private final ElevatorStateManager stateManager;
    private boolean wasStoppedByLimitSwitch = false;

    public ElevatorControlCommand(ElevatorSubsystem elevator, CoralIntakeSubsystem coralIntake, AlgaeReefRemoverSubsystem algaeRemover) {
        this.elevator = elevator;
        this.coralIntake = coralIntake;
        this.algaeRemover = algaeRemover;

        this.stateManager = ElevatorStateManager.INSTANCE;

        addRequirements(elevator, coralIntake, algaeRemover);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // if the elevator height is below a certain height, we need to enable the coral intake's hard bottom limit
        // this means:
        // if the target state needs the coral pivot to go behind the "hard" limit while the elevator is going up, we need to wait to set the pivot target until we've gone up enough
        // if the target state needs to bring the coral pivot from behind the "hard" limit to the safe zone while going down, there's a few options:
        // 1. YOLO and hope it moves fast enough
        // 2. Move the pivot first, and queue the elevator move once the pivot has passed the limit
        // 3. A mix of 1 and 2, where we only do 2 if the elevator is low enough that we're worried it won't pass the limit in time

        // we have chosen option 1 because it turns out the hard limit when we're up is the same as the perma hard limit anyways so no extra code. thanks timothy

        // if we're below the height limit, clamp pivot angle minimum (note: max, global min is clamped in the subsystem)
        double targetHeight = this.stateManager.getHeight();
        double targetPivotAngle = this.stateManager.getPivotAngle().getRotations();

        targetPivotAngle = Math.max(targetPivotAngle, BACK_LIMIT_WHEN_DOWN);
        System.out.println("Target height: " + targetHeight);
        this.elevator.setTargetHeight(targetHeight);
        this.coralIntake.setPivotTargetAngle(Rotation2d.fromRotations(targetPivotAngle));

        if (this.stateManager.getRunAlgaeRemover()) {
            this.algaeRemover.setIntakeSpeed(0.2);
        } else {
            this.algaeRemover.setIntakeSpeed(0);
        }

        switch (this.stateManager.getCoralIntakeState()) {
            case STOPPED:
                this.coralIntake.setIntakeSpeed(0);
                break;
            case INTAKE_FORCE_02:
                this.coralIntake.setIntakeSpeed(0.2);
                break;
            case INTAKE_FORCE_035:
                this.coralIntake.setIntakeSpeed(0.35);
                break;
            case INTAKE:
                if (!wasStoppedByLimitSwitch && this.coralIntake.hasCoral()) { // limit switch pauser
                    this.stateManager.getCurrentState().copy()
                            .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
                            .setAsCurrent();
                    this.wasStoppedByLimitSwitch = true;
                } else if (!this.coralIntake.hasCoral()) {
                    this.wasStoppedByLimitSwitch = false;
                }

                if (!this.coralIntake.hasCoral()) {
                    this.coralIntake.setIntakeSpeed(0.2);
                } else {
                    this.coralIntake.setIntakeSpeed(0.35);
                }
                break;
            case OUTTAKE:
                this.coralIntake.setIntakeSpeed(-0.6);
                break;
            default:
                System.out.println("ElevatorStateManager.ElevatorState has invalid entries for CoralIntakeState that aren't implemented");
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

// at 0 alga is about 0.79 m above floor
// level 1 0.790 meter
// level 2 1.194 meter
// level 3 1.809 meter (also dont forget is like vertical pipe)