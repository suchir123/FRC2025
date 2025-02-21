package frc.robot.commands.testers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class TestCoralIntakePIDCommand extends Command {
    private final CoralIntakeSubsystem coralIntake;

    public TestCoralIntakePIDCommand(CoralIntakeSubsystem coralIntake) {
        this.coralIntake = coralIntake;

        addRequirements(coralIntake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        System.out.println("Setting coral intake pivot angle");
        this.coralIntake.setPivotTargetAngle(Rotation2d.fromRotations(0.5));
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
