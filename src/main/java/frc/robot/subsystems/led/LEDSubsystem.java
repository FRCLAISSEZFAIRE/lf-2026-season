package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

/**
 * LED alt sistemi.
 * AddressableLED (WS2812B) kontrol eder.
 */
public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    // Mevcut durum
    private LEDState currentState = LEDState.IDLE;

    // Animasyon için
    private int rainbowFirstPixelHue = 0;
    private boolean blinkOn = true;
    private double lastBlinkTime = 0;

    public enum LEDState {
        OFF,
        IDLE,
        INTAKING,
        HAS_GAME_PIECE,
        SHOOTING,
        CLIMBING,
        ERROR,
        RAINBOW,
        TEAM_COLOR
    }

    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.kLEDPort);
        buffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case OFF:
                setAllColor(LEDConstants.kColorOff);
                break;
            case IDLE:
                setAllColor(LEDConstants.kColorIdle);
                break;
            case INTAKING:
                blinkColor(LEDConstants.kColorIntaking);
                break;
            case HAS_GAME_PIECE:
                setAllColor(LEDConstants.kColorHasGamePiece);
                break;
            case SHOOTING:
                blinkColor(LEDConstants.kColorShooting);
                break;
            case CLIMBING:
                blinkColor(LEDConstants.kColorClimbing);
                break;
            case ERROR:
                blinkColor(LEDConstants.kColorError);
                break;
            case RAINBOW:
                applyRainbow();
                break;
            case TEAM_COLOR:
                setAllColor(LEDConstants.kColorTeam);
                break;
        }
        led.setData(buffer);
    }

    // ==================== STATE CONTROL ====================

    public void setState(LEDState state) {
        this.currentState = state;
    }

    public LEDState getState() {
        return currentState;
    }

    // ==================== PATTERNS ====================

    private void setAllColor(int[] rgb) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
        }
    }

    private void blinkColor(int[] rgb) {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (currentTime - lastBlinkTime > LEDConstants.kBlinkIntervalSeconds) {
            blinkOn = !blinkOn;
            lastBlinkTime = currentTime;
        }

        if (blinkOn) {
            setAllColor(rgb);
        } else {
            setAllColor(LEDConstants.kColorOff);
        }
    }

    private void applyRainbow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
            buffer.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue = (rainbowFirstPixelHue + 3) % 180;
    }

    // ==================== CONVENIENCE METHODS ====================

    public void off() {
        setState(LEDState.OFF);
    }

    public void setIdle() {
        setState(LEDState.IDLE);
    }

    public void setIntaking() {
        setState(LEDState.INTAKING);
    }

    public void hasGamePiece() {
        setState(LEDState.HAS_GAME_PIECE);
    }

    public void shooting() {
        setState(LEDState.SHOOTING);
    }

    public void climbing() {
        setState(LEDState.CLIMBING);
    }

    public void error() {
        setState(LEDState.ERROR);
    }

    public void rainbow() {
        setState(LEDState.RAINBOW);
    }

    public void teamColor() {
        setState(LEDState.TEAM_COLOR);
    }

    /** Manuel renk ayarlama */
    public void setColor(Color color) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
        led.setData(buffer);
    }
}
