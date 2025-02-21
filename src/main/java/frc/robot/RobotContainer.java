package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeGroundIntakeCommand;
import frc.robot.commands.BalanceClimberCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.elevator.ElevatorControlCommand;
import frc.robot.commands.elevator.ElevatorStateManager;
import frc.robot.commands.elevator.ElevatorStateManager.CoralIntakeState;
import frc.robot.commands.testers.*;
import frc.robot.controllers.AbstractController;
import frc.robot.controllers.NintendoProController;
import frc.robot.controllers.PS4Controller;
import frc.robot.controllers.PS5Controller;
import frc.robot.subsystems.*;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.ControlHandler;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.Util;
import frc.robot.util.ControlHandler.TriggerType;

public class RobotContainer {
    //private static final GenericPublisher COLOR_SENSOR_PUB = NetworkTablesUtil.getPublisher("robot", "color_sensor_sees_note", NetworkTableType.kBoolean);

    /*YAGSL Variables 
    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    SwerveDrive swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed)();
    */

    //private final FlightJoystick sideJoystick = new FlightJoystick(new CommandJoystick(OperatorConstants.RIGHT_JOYSTICK_PORT));
    private final NintendoProController nintendoProController = new NintendoProController(new CommandXboxController(OperatorConstants.NINTENDO_PRO_CONTROLLER));
    private final PS5Controller ps5Controller = new PS5Controller(new CommandPS5Controller(OperatorConstants.PS5_CONTROLLER));
    private final AbstractController primaryController = Flags.Operator.NINTENDO_SWITCH_CONTROLLER_AS_PRIMARY ? this.nintendoProController : this.ps5Controller;
    private final PS4Controller ps4Controller = new PS4Controller(new CommandPS4Controller(OperatorConstants.PS4_CONTROLLER));
    private final PowerHandler powerHandler = new PowerHandler();
    // private final AprilTagHandler aprilTagHandler = new AprilTagHandler();

    // private final SendableChooser<Command> autonChooser;

    private final DriveTrainSubsystem driveTrain;
    private final ElevatorSubsystem elevators;
    private final ClimberSubsystem climber;
    private final CoralIntakeSubsystem coralIntake;
    private final AlgaeReefRemoverSubsystem algaeReefRemover;
    private final AlgaeGroundIntakeSubsystem algaeGroundIntake;

    private final ElevatorStateManager elevatorStateManager = ElevatorStateManager.INSTANCE;

    public RobotContainer() {
        this.driveTrain = Util.createIfFlagElseNull(DriveTrainSubsystem::new, Flags.DriveTrain.IS_ATTACHED);
        this.elevators = Util.createIfFlagElseNull(ElevatorSubsystem::new, Flags.Elevator.IS_ATTACHED);
        this.climber = Util.createIfFlagElseNull(ClimberSubsystem::new, Flags.Climber.IS_ATTACHED);
        this.coralIntake = Util.createIfFlagElseNull(CoralIntakeSubsystem::new, Flags.CoralIntake.IS_ATTACHED);
        this.algaeReefRemover = Util.createIfFlagElseNull(AlgaeReefRemoverSubsystem::new, Flags.AlgaeReefRemover.IS_ATTACHED);
        this.algaeGroundIntake = Util.createIfFlagElseNull(AlgaeGroundIntakeSubsystem::new, Flags.AlgaeGroundIntake.IS_ATTACHED);

        configureBindings();

        // Initialize static subsystems (this is a Java thing don't worry about it just copy it so that static blocks run on startup)
        LimeLight.poke();
        RobotGyro.poke();
        // ColorSensor.poke();

        // if (Flags.DriveTrain.IS_ATTACHED && Flags.DriveTrain.ENABLE_AUTON_CHOOSER) {
        //     this.autonChooser = AutoBuilder.buildAutoChooser();
        //     SmartDashboard.putData("choose your auto", this.autonChooser);
        // } else {
        //     this.autonChooser = null;
        // }

        NetworkTablesUtil.getConnections();
    }

    private void configureBindings() {
        if(Flags.Elevator.IS_ATTACHED && !Flags.Elevator.USE_TEST_ELEVATOR_COMMAND && !Flags.Elevator.USE_TEST_PID_COMMAND && !Flags.CoralIntake.USE_TEST_PID_COMMAND) {
            ControlHandler.get(this.ps4Controller, OperatorConstants.SecondaryControllerConstants.INTAKE_STATE).onTrue(new InstantCommand(() -> elevatorStateManager.cloneState()
                    .setHeight(0)
                    .setPivotAngle(Rotation2d.fromRotations(0.14))
                    .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
                    .setRunAlgaeRemover(false)
                    .setAsCurrent()));

            ControlHandler.get(this.ps4Controller, OperatorConstants.SecondaryControllerConstants.L1).onTrue(new InstantCommand(() -> elevatorStateManager.cloneState()
                    .setHeight(0)
                    .setPivotAngle(Rotation2d.fromRotations(0.03))
                    .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
                    .setRunAlgaeRemover(false)
                    .primeAsNext()));

            ControlHandler.get(this.ps4Controller, OperatorConstants.SecondaryControllerConstants.L2).onTrue(new InstantCommand(() -> elevatorStateManager.cloneState()
                    .setHeight(0)
                    .setPivotAngle(Rotation2d.fromRotations(0.46))
                    .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
                    .setRunAlgaeRemover(false)
                    .primeAsNext()));

            ControlHandler.get(this.ps4Controller, OperatorConstants.SecondaryControllerConstants.L3).onTrue(new InstantCommand(() -> elevatorStateManager.cloneState()
                    .setHeight(0.39)
                    .setPivotAngle(Rotation2d.fromRotations(0.46))
                    .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
                    .setRunAlgaeRemover(false)
                    .primeAsNext()));

            ControlHandler.get(this.ps4Controller, OperatorConstants.SecondaryControllerConstants.L4).onTrue(new InstantCommand(() -> elevatorStateManager.cloneState()
                    .setHeight(1.02)
                    .setPivotAngle(Rotation2d.fromRotations(0.45))
                    .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.STOPPED)
                    .setRunAlgaeRemover(false)
                    .primeAsNext()));

            ControlHandler.get(this.primaryController, TriggerType.LOWER_BUTTON).onTrue(new InstantCommand(() -> elevatorStateManager.cloneState()
                    .setHeight(0)
                    .setPivotAngle(Rotation2d.fromRotations(0.35))
                    .setCoralIntakeState(CoralIntakeState.STOPPED)
                    .setRunAlgaeRemover(false)
                    .setAsCurrent()
            ));

            /*
            ControlHandler.get(this.ps4Controller, OperatorConstants.SecondaryControllerConstants.APPLY_ALGAE_REMOVER_HEIGHT_DELTA).onTrue(new InstantCommand(() -> elevatorStateManager.cloneState()
                    .setHeight(elevatorStateManager.getHeight() + 0.2)
                    .setAsCurrent()))
            .onChange(new InstantCommand(() -> elevatorStateManager.cloneState()
                    .setHeight(elevatorStateManager.getHeight() - 0.1).setAsCurrent()
            ));*/

            ControlHandler.get(this.primaryController, OperatorConstants.PrimaryControllerConstants.CORAL_INTAKE_MOTOR).onTrue(new InstantCommand(() -> elevatorStateManager.cloneState()
                    .setCoralIntakeState(ElevatorStateManager.CoralIntakeState.INTAKE)
                    .setAsCurrent()));

            ControlHandler.get(this.primaryController, OperatorConstants.PrimaryControllerConstants.ALGAE_REMOVER).onTrue(new InstantCommand(() -> elevatorStateManager.cloneState()
                    .setHeight(elevatorStateManager.getHeight() + (!this.elevatorStateManager.getRunAlgaeRemover() ? 0.11 : -0.11))
                    .setRunAlgaeRemover(!this.elevatorStateManager.getRunAlgaeRemover())
                    .setAsCurrent()));

            ControlHandler.get(this.primaryController, OperatorConstants.PrimaryControllerConstants.ACTIVATE_ELEVATORS).onTrue(new InstantCommand(elevatorStateManager::pushNextState));

            if(Flags.DriveTrain.IS_ATTACHED) {
                ControlHandler.get(this.ps4Controller, OperatorConstants.SecondaryControllerConstants.RESET_GYRO).onTrue(new InstantCommand(() -> {
                    if(Util.onBlueTeam()) {
                        RobotGyro.resetGyroAngle();
                    } else {
                        RobotGyro.setGyroAngle(180);
                    }
                    this.driveTrain.setHeadingLockMode(false);
                }));
            }
        }

        // TODO: bindings need to be re-implemented if still in use for YAGSL drive
        // These old bindings are only for the nintendo pro controller, which we no longer use.


        // if (Flags.DriveTrain.IS_ATTACHED) {
        //     ControlHandler.get(this.nintendoProController, ControllerConstants.ZERO_SWERVE_MODULES).onTrue(this.driveTrain.rotateToAbsoluteZeroCommand());
        // }
        // ControlHandler.get(this.nintendoProController, ControllerConstants.ZERO_GYRO).onTrue(Commands.runOnce(() -> {
        //     if(Util.onBlueTeam()) {
        //         RobotGyro.resetGyroAngle();
        //     } else {
        //         RobotGyro.setGyroAngle(180);
        //     }
        //     // TODO: implement heading lock mode on YAGSL drive
        //     //this.driveTrain.setHeadingLockMode(false);
        // }));
        // if(Flags.DriveTrain.IS_ATTACHED) {
        //     ControlHandler.get(this.nintendoProController, ControllerConstants.RESET_POSE_ESTIMATOR).onTrue(new InstantCommand(this.driveTrain::resetPoseToMidSubwoofer));
        // }
    }

    public void onRobotInit() {
        FlagUploader.uploadFlagsClass();
    }

    public void onTeleopInit() {
        if (this.getAutonomousCommand() != null) {
            this.getAutonomousCommand().cancel();
        }

        if (Flags.Elevator.IS_ATTACHED) {
            if (Flags.Elevator.USE_TEST_ELEVATOR_COMMAND) {
                this.elevators.setDefaultCommand(new TestElevatorCommand(this.elevators, this.primaryController));
            } else if (Flags.Elevator.USE_TEST_PID_COMMAND) {
                this.elevators.setDefaultCommand(new TestElevatorPIDCommand(this.elevators, this.primaryController));
            }
        }
        if (Flags.DriveTrain.IS_ATTACHED) {
            if (Flags.DriveTrain.USE_TEST_DRIVE_COMMAND) {
                this.driveTrain.setDefaultCommand(new TestDriveCommand(this.driveTrain, this.primaryController));
            } else {
                this.driveTrain.setDefaultCommand(new ManualDriveCommand(this.driveTrain, this.primaryController));
            }
        }

        if (Flags.Climber.IS_ATTACHED) {
            if (Flags.Climber.USE_TEST_CLIMBER_COMMAND) {
                this.climber.setDefaultCommand(new TestClimberCommand(climber, this.primaryController));
            } else {
                Command autoClimb = new ClimbCommand(climber).andThen(new BalanceClimberCommand(climber));
                // ControlHandler.get(this.primaryController, Constants.OperatorConstants.SecondaryControllerConstants.AUTO_CLIMB).whileTrue(autoClimb);
            }
        }

        if (Flags.CoralIntake.IS_ATTACHED) {
            if (Flags.CoralIntake.USE_TEST_CORAL_COMMAND) {
                this.coralIntake.setDefaultCommand(new TestCoralIntakeCommand(coralIntake, this.primaryController));
            } else if(Flags.CoralIntake.USE_TEST_PID_COMMAND) {
                this.coralIntake.setDefaultCommand(new TestCoralIntakePIDCommand(coralIntake));
            }
        }

        if(Flags.Elevator.IS_ATTACHED && Flags.CoralIntake.IS_ATTACHED && Flags.AlgaeReefRemover.IS_ATTACHED && !Flags.Elevator.USE_TEST_PID_COMMAND && !Flags.Elevator.USE_TEST_ELEVATOR_COMMAND && !Flags.CoralIntake.USE_TEST_PID_COMMAND && !Flags.CoralIntake.USE_TEST_CORAL_COMMAND && !Flags.AlgaeReefRemover.USE_TEST_ALGAE_REMOVER_COMMAND) {
            this.elevators.setDefaultCommand(new ElevatorControlCommand(this.elevators, this.coralIntake, this.algaeReefRemover));
        }

        if(Flags.AlgaeGroundIntake.IS_ATTACHED) {
            if(Flags.AlgaeGroundIntake.USE_TEST_ALGAE_GROUND_COMMAND) {
            } else {
                this.algaeGroundIntake.setDefaultCommand(new AlgaeGroundIntakeCommand(this.algaeGroundIntake, this.primaryController, this.ps4Controller));
            }
        }

        /* // put another slash to undestroy this
        if (Flags.Climber.IS_ATTACHED) {
            if (Flags.Climber.USE_TEST_CLIMBER_COMMAND) {
                this.climber.setDefaultCommand(new TestClimberCommand(this.climber, this.primaryController));
            } else {
                this.climber.setDefaultCommand(new ManualClimberCommand(this.climber, this.primaryController));
            }
        }
        if (Flags.CoralIntake.IS_ATTACHED) {
            if (Flags.CoralIntake.USE_TEST_CORAL_COMMAND) {
                this.coralIntake.setDefaultCommand(new TestCoralIntakeCommand(this.coralIntake, this.primaryController));
            } else {
                this.coralIntake.setDefaultCommand(new ManualCoralIntakeCommand(this.coralIntake, this.primaryController));
            }
        }
        // */
    }

    public Command getAutonomousCommand() {
        // This method will return an actual auton path once we implement it & switch in the comment.
        return new InstantCommand(); //this.autonChooser.getSelected();
    }

    public void onTeleopPeriodic() {
        // this.powerHandler.updateNT();
    }

    public void onRobotPeriodic() {

    }
}
