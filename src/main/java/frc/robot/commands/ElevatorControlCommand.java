package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeReefRemoverSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorControlCommand extends Command {
    public enum CoralIntakeState {
        STOPPED,
        INTAKE,
        OUTTAKE
    }

    public record ElevatorState(double height, double pivotAngle, CoralIntakeState coralIntakeState, boolean runAlgaeRemover) {
    }

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
        this.elevator.setTargetHeight(this.targetState.height());
        this.coralIntake.setPivotTargetAngle(this.targetState.pivotAngle());
        if(this.targetState.runAlgaeRemover()) {
            this.algaeRemover.setRawSpeed(0.2);
        }
        switch(this.targetState.coralIntakeState()) {
            case STOPPED:
                this.coralIntake.setRawSpeed(0);
                break;
            case INTAKE:
                this.coralIntake.setRawSpeed(0.2);
                break;
            case OUTTAKE:
                this.coralIntake.setRawSpeed(-0.2);
                break;
            default:
                System.out.println("ElevatorControlCommand.ElevatorState has invalid entries that aren't accounted for");
        }
    }

    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.algaeRemover.setRawSpeed(0);
        this.coralIntake.setRawSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}// at 0 alga is about 0.79 m above floor
// level 1 0.790 meter
// level 2 1.194 meter
// level 3 1.809 meter (also dont forget is like vertical pipe)
//
