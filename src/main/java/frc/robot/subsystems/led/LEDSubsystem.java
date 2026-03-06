package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.feeder.FeederSubsystem;

/**
 * LED alt sistemi.
 * (15 Sol + 15 Sağ = 30 LED)
 */
public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final FeederSubsystem feeder;

    private static final int STRIP_LENGTH = 15;
    private static final int TOTAL_LENGTH = RobotMap.kLEDLength; // 30

    // Durumlar
    private int rainbowFirstPixelHue = 0;
    private boolean isShooting = false;

    public LEDSubsystem(FeederSubsystem feeder) {
        this.feeder = feeder;

        led = new AddressableLED(RobotMap.kLEDPort);
        buffer = new AddressableLEDBuffer(TOTAL_LENGTH);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    public void setShooting(boolean shooting) {
        this.isShooting = shooting;
    }

    @Override
    public void periodic() {
        if (isShooting) {
            // ATIŞ MODU -> GÖKKUŞAĞI ANİMASYONU
            rainbow();
        } else {
            // Otomatik Durum Kontrolü (Feeder)
            updateBasedOnFeeder();
        }

        // Veriyi gönder
        led.setData(buffer);
    }

    /**
     * Feeder durumuna göre LED'leri güncelle
     */
    private void updateBasedOnFeeder() {
        // Alliance rengini göster
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                setAll(Color.kRed);
            } else {
                setAll(Color.kBlue);
            }
        } else {
            setAll(Color.kBlack); // Off
        }
    }

    /** Tüm şeridi aynı renge boya */
    public void setAll(Color color) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }

    /** Sol şeridi boya (0-14) */
    public void setLeft(Color color) {
        for (int i = 0; i < STRIP_LENGTH; i++) {
            buffer.setLED(i, color);
        }
    }

    /** Sağ şeridi boya (15-29) */
    public void setRight(Color color) {
        for (int i = STRIP_LENGTH; i < TOTAL_LENGTH; i++) {
            buffer.setLED(i, color);
        }
    }

    /** Gökkuşağı Efekti */
    public void rainbow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
            buffer.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue = (rainbowFirstPixelHue + 3) % 180;
    }
}
