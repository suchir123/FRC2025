package frc.robot.commands.testers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.elevator.ElevatorStateManager;
import frc.robot.commands.elevator.ElevatorStateManager.CoralIntakeState;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.ClimberSubsystem;

public class TestClimbPIDCommand extends Command {
    private final ClimberSubsystem climber;
    private final AbstractController controller;

    private final Trigger uB, leftB, lowB, rB;

    public TestClimbPIDCommand(ClimberSubsystem climber, AbstractController c) {
        this.climber = climber;
        this.controller = c;

        uB = c.pov(0);
        leftB = c.pov(270);
        lowB = c.pov(180);
        rB = c.pov(90);

        addRequirements(climber);
    }

    @Override
    public void initialize() {
    }

    private boolean wasPID = false;
    @Override
    public void execute() {
        if (controller.getPOV() == 0) {
            wasPID = false;
            climber.setRawSpeed(0.4);
        } else if (controller.getPOV() == 180) {
            wasPID = false;
            climber.setRawSpeed(-0.4);
        } else if(controller.getPOV() == 90) {
            wasPID = true;
            climber.setTargetRotationCount(0.66);
            ElevatorStateManager.INSTANCE.cloneState()
                    .setHeight(0)
                    .setPivotAngle(Rotation2d.fromRotations(0.4))
                    .setCoralIntakeState(CoralIntakeState.STOPPED)
                    .setRunAlgaeRemover(false)
                    .setAsCurrent();
        } else if(controller.getPOV() == 270) {
            wasPID = true;
            climber.setTargetRotationCount(0.535);
        } else if(!wasPID) {
            climber.setRawSpeed(0);
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
}// at 0 alga is about 0.79 m above floor
// level 1 0.790 meter
// level 2 1.194 meter
// level 3 1.809 meter (also dont forget is like vertical pipe)
//
