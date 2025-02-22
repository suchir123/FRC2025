package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.Flags;

import java.lang.reflect.Field;

public class PowerHandler {
    private static final DoublePublisher voltagePublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("battery_voltage").publish();
    private static final DoublePublisher currentPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("total_current").publish();

    private final PowerDistribution pdp;

    public PowerHandler() {
        this.pdp = new PowerDistribution();

        if (Flags.Debug.PRINT_PDP_STICKY_FAULTS) {
            this.printStickyFaults();
        }
        if (Flags.Debug.CLEAR_PDP_STICKY_FAULTS) {
            this.pdp.clearStickyFaults();
        }
    }

    public void printStickyFaults() {
        // Use reflection to print the name of each erroring (true) field of the 
        // pdp sticky faults bitfield.
        var f = pdp.getStickyFaults();
        boolean anyFaultPresent = false;
        try {
            for (Field field : f.getClass().getDeclaredFields()) {
                boolean isFaulting = field.getBoolean(f);
                if (isFaulting) {
                    if (!anyFaultPresent) {
                        anyFaultPresent = true;
                        System.out.println("ERROR! BAD THING! PDP STICKY FAULTS ARE PRESENT!");
                        System.out.println("Faults are:");
                    }
                    System.out.println(field.getName());
                }
            }
        } catch (Exception exception) {
            System.out.println("Error during printing PDP faults:");
            exception.printStackTrace();
        }
    }

    public double getVoltage() {
        return this.pdp.getVoltage();
    }

    public double getTotalCurrent() {
        return this.pdp.getTotalCurrent();
    }

    public double getChannelCurrent(int channel) {
        return this.pdp.getCurrent(channel);
    }

    public void updateNT() {
        voltagePublisher.set(this.getVoltage());
        currentPublisher.set(this.getTotalCurrent());
    }
}
