package frc.robot.util;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Note: The subsystem using this class MUST call ThroughboreEncoder.periodic() once per frame.
 * 
 * original comments when this code was written:
 * new ThroughboreEncoder is pushed to main. some concerns:
 *   - there's a reversed argument to Encoder that we weren't using before, despite having a reversed argument in the ThroughboreEncoder constructor that never got used anywhere. we now pass `reversed` to the RelativeEncoder
 *   - i made absoluteOffset be only used for the AbsoluteEncoder, not the RelativeEncoder, i think this is right
 *   - i'm not sure how the telescoping arm works, but if the motor keeps spinning when we get to the top without actually moving the thing up, that's gonna confuse the ThroughboreEncoder a lot
 *   - i made it return Rotations2ds instead of `double`s, so we need to be careful not to use auto-wrapping operations (like .plus() or .minus()) on the Rotation2d returned by the AbsoluteEncoders to turn the output relative
 *   - we need to call ThroughboreEncoder.periodic() every frame now
 *   - i made an `ThroughboreEncoderInputs` class to prepare for the `AdvantageKit`ing
 */
public class ThroughboreEncoder {
    // Preemptively refactoring for AdvantageKit 
    public class ThroughboreEncoderInputs {
        public double absoluteEncoderDistance = 0;
        public double relativeEncoderDistance = 0;
    }

    private final DutyCycleEncoder absoluteEncoder;
    private final Encoder relativeEncoder;
    private final boolean reversed;
    private final boolean withoutRelativeEncoder;

    private int countFullRotations = 0;
    private final double absoluteOffset;

    private final ThroughboreEncoderInputs inputs = new ThroughboreEncoderInputs();

    public ThroughboreEncoder(int encoderAbsPort, int encoderAPort, int encoderBPort, double absoluteOffset, boolean reversed, boolean withoutRelativeEncoder) {
        this.absoluteEncoder = new DutyCycleEncoder(encoderAbsPort, 1.0, 0.0);
        if (withoutRelativeEncoder) {
            this.relativeEncoder = null;
        } else {
            this.relativeEncoder = new Encoder(encoderAPort, encoderBPort, reversed, EncodingType.k4X);
        }

        this.withoutRelativeEncoder = withoutRelativeEncoder;
        this.reversed = reversed;
        this.absoluteOffset = absoluteOffset;

        if (!this.absoluteEncoder.isConnected()) {
            System.out.println("Critical error: a ThroughboreEncoder's absoluteEncoder is not connected.");
        }
    }

    public ThroughboreEncoder(int encoderAbsPort, int encoderAPort, int encoderBPort) {
        this(encoderAbsPort, encoderAPort, encoderBPort, 0.0, false, false);
    }

    /*
     * Returns the current angle of the shaft. Should be roughly equal to getAbsoluteDistance() % 360deg if absoluteOffset is 0.
     * Does not use the absolute offset passed in.
     */
    public Rotation2d getRelativeEncoderValue() {
        if (withoutRelativeEncoder) {
            throw new UnsupportedOperationException("You called ThroughboreEncoder::getRelativeEncoderValue on me, but"
                + " I was instantiated with withoutRelativeEncoder=true, so I don't have one.");
        }
        return Rotation2d.fromRotations(this.inputs.relativeEncoderDistance);
    }

    /*
     * Returns the total distance (could be negative or greater than 360 degrees) that the encoder has moved since boot.
     * Note: the returned value is a Rotation2d that can have an interior value greater than 360 degrees.
     * This can lead to unwanted behaviour because Rotation methods like add() will also wrap 0-360.
     */
    public Rotation2d getAbsoluteDistance() {
        return Rotation2d.fromRotations(this.countFullRotations + this.inputs.absoluteEncoderDistance + this.absoluteOffset);
    }

    private double getReversedAbsoluteEncoderValue() {
        if (this.reversed) {
            return 1.0 - this.absoluteEncoder.get();
        } else {
            return this.absoluteEncoder.get();
        }
    }

    public void periodic() {
        // Structure here is intentional.
        updateInputs();
    }

    private void updateInputs() {
        double currentAbsoluteEncoderDistance = this.getReversedAbsoluteEncoderValue();
        double changeFromLastOutput = currentAbsoluteEncoderDistance - this.inputs.absoluteEncoderDistance;
        if (changeFromLastOutput <= -0.5) {
            // We just rolled over from a very high value to a low one, finishing a rotation.
            this.countFullRotations += 1;
        } else if (changeFromLastOutput >= 0.5) {
            // We just rolled under from a low value back to a very high one, undoing a full rotation.
            this.countFullRotations -= 1;
        }
        this.inputs.absoluteEncoderDistance = currentAbsoluteEncoderDistance;
        if (!this.withoutRelativeEncoder) {
            this.inputs.relativeEncoderDistance = this.relativeEncoder.get();
        }
    }
}
