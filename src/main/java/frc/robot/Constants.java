package frc.robot;

import frc.robot.util.ControlHandler.TriggerType;

/**
 * Intended for constants on the robot that should rarely change (especially in a competition scenario). These values tend to be primitive values (but are not necessarily).
 * Examples of these values are ports or IDs where motors are plugged in.
 * This is different from the {@link frc.robot.Flags Flags} class, which toggles functionality on the robot and may be changed more often.
 */
public final class Constants {
    private Constants() {
        throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }

    public static class NetworkTablesConstants {
        public static final String MAIN_TABLE_NAME = "robot";
    }

    public static class OperatorConstants { // move it back by 10 cm!!!
        public static final int RIGHT_JOYSTICK_PORT = 2;
        public static final int NINTENDO_PRO_CONTROLLER = 1;
        public static final int PS5_CONTROLLER = 3;
        public static final int PS4_CONTROLLER = 4;

        // Should this be here? especially with our new controller system, we could potentially refactor or re-abstract this using another class (maybe even for multiple driver preferences?)
        public static class PrimaryControllerConstants {
            public static final TriggerType ALGAE_GROUND_INTAKE                 = TriggerType.LEFT_SHOULDER_TRIGGER;
            public static final TriggerType ALGAE_GROUND_OUTTAKE                = TriggerType.LEFT_SHOULDER_BUTTON;
            public static final TriggerType ALGAE_AUTO_AIM                      = TriggerType.LEFT_BUTTON;
            public static final TriggerType CORAL_INTAKE_MOTOR                  = TriggerType.RIGHT_SHOULDER_TRIGGER;
            public static final TriggerType ACTIVATE_ELEVATORS                  = TriggerType.RIGHT_SHOULDER_BUTTON;

            public static final TriggerType ALGAE_REMOVER                       = TriggerType.RIGHT_BUTTON;
            public static final TriggerType REEF_AUTO_AIM                       = TriggerType.UPPER_BUTTON;
        }

        public static class SecondaryControllerConstants {
            public static final TriggerType RESET_GYRO                          = TriggerType.POV_0;
            public static final TriggerType TOGGLE_ALGAE_GROUND_INTAKE_HEIGHT   = TriggerType.LEFT_SHOULDER_BUTTON;

            public static final TriggerType L4                                  = TriggerType.RIGHT_SHOULDER_BUTTON;
            public static final TriggerType L3                                  = TriggerType.UPPER_BUTTON;
            public static final TriggerType L2                                  = TriggerType.LEFT_BUTTON;
            public static final TriggerType L1                                  = TriggerType.LOWER_BUTTON;
            public static final TriggerType INTAKE_STATE                        = TriggerType.RIGHT_BUTTON;
        }
    }

    public static class PortConstants {
        /**
         * Key: RIO = RoboRio, COD = CANCoder, DRI = Drive Motor, ROT = Rotation Motor, ELE = Elevator, CLI = Climber
         * <pre>
         * CAN IDs USED:
         * 0  (RIO)
         *
         * 1  (COD)
         * 2  (ROT)
         * 3  (COD)
         * 4  (DRI)
         * 5  (ROT)
         * 6  (COD)
         * 7  (DRI)
         * 8  (ROT)
         * 9  (DRI)
         * 10 (ROT)
         * 11 (DRI)
         * 12 (COD)
         *
         * 17 (ELE)
         *
         * 21 (ELE)
         */
        public static class CAN {
            // Drive Train (COD, DRI, ROT)
            public static final int DTRAIN_FRONT_LEFT_DRIVE_MOTOR_ID = 11;
            public static final int DTRAIN_FRONT_RIGHT_DRIVE_MOTOR_ID = 21;
            public static final int DTRAIN_BACK_LEFT_DRIVE_MOTOR_ID = 14;
            public static final int DTRAIN_BACK_RIGHT_DRIVE_MOTOR_ID = 19;
            public static final int DTRAIN_FRONT_LEFT_ROTATION_MOTOR_ID = 10;
            public static final int DTRAIN_FRONT_RIGHT_ROTATION_MOTOR_ID = 22;
            public static final int DTRAIN_BACK_LEFT_ROTATION_MOTOR_ID = 15;
            public static final int DTRAIN_BACK_RIGHT_ROTATION_MOTOR_ID = 20;
            public static final int DTRAIN_FRONT_LEFT_CANCODER_ID = 12;
            public static final int DTRAIN_FRONT_RIGHT_CANCODER_ID = 3;
            public static final int DTRAIN_BACK_LEFT_CANCODER_ID = 6;
            public static final int DTRAIN_BACK_RIGHT_CANCODER_ID = 1;

            // (2025) Elevators (ELE)
            public static final int RIGHT_ELEVATOR_MOTOR_ID = 7;
            public static final int LEFT_ELEVATOR_MOTOR_ID = 16;

            // (2025) Climber
            public static final int CLIMBER_MOTOR_ID = 5;

            // (2025) Coral Intake
            public static final int CORAL_PIVOT_MOTOR_ID = 13;
            public static final int CORAL_INTAKE_MOTOR_ID = 4;

            // (2025) Algae Remover mounted on elevator
            public static final int ALGAE_REMOVER_MOTOR_ID = 2;

            // (2025) Algae Ground Intake
            public static final int ALGAE_GROUND_INTAKE_MOTOR_ID = 23;
        }
//ivan is very smart and he is our very bestest diversity hire as the only white man in our team besides mr. reid and mr. goodman and we all love ivan because we have asianified him real good because he loves eating rice and filters his water at home instead of drinking tap water and hes real good coder ivan also cant eat wheat because hes celiac so as a white guy he cant eat white bread ivan can also use chopsticks real good when we eate food at like 12 that time he matched his photo with a frog (but he didnt want to match with the girl frog) he also has lots of good shimp photos which make him look very shrimpy with his long shrimpy nose and large shrimpy eyes
        public static class PWM {
            public static final int ALGAE_LEFT_SERVO_PORT = 1;
            public static final int ALGAE_RIGHT_SERVO_PORT = 0;
        }

        /**
         * ELE = Elevator, CLI = Climber
         * <pre>
         * DIOs USED:
         * 0 (ELE)
         * 1 (ELE)
         * 2 (ELE)
         * 3 (ELE)
         * 4 (ELE)
         * 5 (ELE)
         */
        public static class DIO {
            public static final int LEFT_ELEVATOR_LIMIT = 0;
            public static final int RIGHT_ELEVATOR_LIMIT = 1;
        }
    }

    public static class DriveConstants {
        public static final double TRAJ_X_CONTROLLER_KP = 0;
        public static final double TRAJ_Y_CONTROLLER_KP = 0;
        public static final double TRAJ_THETA_CONTROLLER_KP = 0;
        public static final double TRAJ_MAX_ANG_VELO = 0;
        public static final double TRAJ_MAX_ANG_ACCEL = 0;

        public static final double MAX_SPEED = 1.5;
        public static final double ANGULAR_VELOCITY_COEFF = 0.1; // YAGSL
    }

    public static class RobotConstants {
        public static final double SIDE_LENGTH_INCHES = 15; // square

        public static final double DIAGONAL_LENGTH_INCHES = 1.41421356 * SIDE_LENGTH_INCHES; // sqrt(2)
        public static final double DIAGONAL_LENGTH_CM = DIAGONAL_LENGTH_INCHES * 2.54;
        public static final double SWERVE_MODULE_INSET_FROM_CORNER_CM = 9; // CM
        public static final double SWERVE_MODULE_DIST_FROM_MIDDLE_CM = DIAGONAL_LENGTH_CM - SWERVE_MODULE_INSET_FROM_CORNER_CM;
        public static final double LEG_LENGTHS_CM = SWERVE_MODULE_DIST_FROM_MIDDLE_CM / 1.41421356; // sqrt(2)
        public static final double LEG_LENGTHS_M = LEG_LENGTHS_CM / 100;
    }
}
