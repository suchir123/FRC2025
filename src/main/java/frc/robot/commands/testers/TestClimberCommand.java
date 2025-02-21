package frc.robot.commands.testers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.ClimberSubsystem;

public class TestClimberCommand extends Command {
    private final ClimberSubsystem climber;
    private final AbstractController joystick;

    public TestClimberCommand(ClimberSubsystem climber, AbstractController joystick) {
        this.climber = climber;
        this.joystick = joystick;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(joystick.getPOV() == 0) {
            climber.setRawSpeed(0.35);
        } else if(joystick.getPOV() == 180) {
            climber.setRawSpeed(-0.35);
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
}
