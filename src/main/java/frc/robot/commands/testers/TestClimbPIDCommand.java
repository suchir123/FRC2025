package frc.robot.commands.testers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class TestClimbPIDCommand extends Command {
    private final ClimberSubsystem climber;
    private final AbstractController controller;

    private final Trigger uB, leftB, lowB, rB;

    public TestClimbPIDCommand(ClimberSubsystem climber, AbstractController c) {
        this.climber = climber;
        this.controller = c;

        uB = c.pov(0);
        leftB = c.pov(90);
        lowB = c.pov(180);
        rB = c.pov(270);

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (controller.getPOV() == 0) {
            climber.setRawSpeed(0.7);
        } else if (controller.getPOV() == 180) {
            climber.setRawSpeed(-0.7);
        } else if(rB.getAsBoolean()) {
            climber.setTargetRotationCount(0.82);
        } else if(leftB.getAsBoolean()) {
            climber.setTargetRotationCount(0.68);
        } else {
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
