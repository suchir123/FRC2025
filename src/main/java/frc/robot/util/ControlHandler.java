package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.AbstractController;

public final class ControlHandler {
    /**
     * Gets the {@link Trigger} from the given controller based on the requested button.
     * <p>
     * IMPORTANT: THIS METHOD SHOULD NEVER BE CALLED PER-TICK (i.e. in execute() or periodic() methods) AS IT WILL CREATE EXTRANEOUS OBJECTS. PREFER TO STORE ONE INSTANCE OF THE TRIGGER INSTEAD.
     * IMPORTANT: TRIGGERS SHOULD NOT BIND ITEMS IN THE EXECUTE() METHOD. THIS WILL ALSO CREATE EXTRANEOUS OBJECTS. PREFER TO BIND IN initialize().
     *
     * @param controller The controller to get the Trigger from.
     * @param type       The button to get.
     * @return The Trigger for the requested button from the given controller.
     */
    public static Trigger get(AbstractController controller, TriggerType type) {
        return switch (type) {
            case UPPER_BUTTON -> controller.upperButton();
            case LEFT_BUTTON -> controller.leftButton();
            case RIGHT_BUTTON -> controller.rightButton();
            case LOWER_BUTTON -> controller.lowerButton();
            case LEFT_SHOULDER_BUTTON -> controller.leftShoulderButton();
            case RIGHT_SHOULDER_BUTTON -> controller.rightShoulderButton();
            case LEFT_SHOULDER_TRIGGER -> controller.leftShoulderTrigger();
            case RIGHT_SHOULDER_TRIGGER -> controller.rightShoulderTrigger();
            case POV_0 -> controller.pov(0);
            case POV_45 -> controller.pov(45);
            case POV_90 -> controller.pov(90);
            case POV_135 -> controller.pov(135);
            case POV_180 -> controller.pov(180);
            case POV_225 -> controller.pov(225);
            case POV_270 -> controller.pov(270);
            case POV_315 -> controller.pov(315);
        };
    }

    public enum TriggerType {
        UPPER_BUTTON,
        LEFT_BUTTON,
        RIGHT_BUTTON,
        LOWER_BUTTON,
        LEFT_SHOULDER_BUTTON,
        RIGHT_SHOULDER_BUTTON,
        LEFT_SHOULDER_TRIGGER,
        RIGHT_SHOULDER_TRIGGER,
        POV_0,
        POV_45,
        POV_90,
        POV_135,
        POV_180,
        POV_225,
        POV_270,
        POV_315
    }

    private ControlHandler() {}
}
