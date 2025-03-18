package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

import java.util.HashMap;
import java.util.Map;

/*
 * Handles inputs/outputs to an LED strip.
 * Does not need to be updated every frame.
 */
public class LEDStrip {
	private static final AddressableLED led;
	private static final AddressableLEDBuffer ledBuffer;
	// Note: Don't make a bunch of views and mess up our nice hashmap.
	private final AddressableLEDBufferView bufferView;
	
	private static final LEDPattern on  = LEDPattern.solid(Color.kWhite);
	private static final LEDPattern off = LEDPattern.kOff;
	
	static {
		led = new AddressableLED(Constants.PortConstants.PWM.LED_PORT);
		ledBuffer = new AddressableLEDBuffer(Constants.RobotConstants.LED_LENGTH);
		
		led.setLength(ledBuffer.getLength());
		led.setData(ledBuffer);
		led.start();
	}
	
	public static void poke(){}
	
	public LEDStrip(int start, int end) {
		this.bufferView = ledBuffer.createView(start, end);
	}
	
	public void turnOn()
	{
		usePattern(on);
	}
	
	public void turnOff()
	{
		usePattern(off);
	}
	
	public void usePattern(LEDPattern pattern)
	{
		pattern.applyTo(this.bufferView);
	}
	
	private AddressableLEDBufferView createBufferView(int start, int end) {
		return ledBuffer.createView(start, end);
	}
	
	public static void update() {
		led.setData(ledBuffer);
	}
}
