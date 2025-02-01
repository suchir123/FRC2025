package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.util.Util;

public class TestElevatorCommand extends Command {
    private final TelescopingArm telescopingArm;
    private final AbstractController joystick;

    public TestElevatorCommand(TelescopingArm telescopingArm, AbstractController joystick) {
        this.telescopingArm = telescopingArm;
        this.joystick = joystick;

        addRequirements(telescopingArm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double rightSpeed = this.joystick.getRightVerticalMovement();
        double leftSpeed  = this.joystick.getLeftVerticalMovement();

        System.out.println("Right speed = " + rightSpeed + "\n" + 
                           "Left speed = " + leftSpeed + "\n" + 
                           "Right position = " + this.telescopingArm.getLeadingThroughboreEncoderDistance() + "\n" + 
                           "Left position = " + this.telescopingArm.getFollowingThroughboreEncoderDistance());

        // right is lead, left is follow.
        // this.telescopingArm.ascendSimple(rightSpeed, leftSpeed);
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
