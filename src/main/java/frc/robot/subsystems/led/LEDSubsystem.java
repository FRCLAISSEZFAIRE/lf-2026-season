package frc.robot.subsystems.led;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotMap;

import java.util.function.Supplier;

/**
 * LED alt sistemi — Animasyonlu durum göstergesi.
 * 
 * Durumlar (öncelik sırasına göre):
 * 1. SHOOTING → Hızlı beyaz strobe / flash
 * 2. INTAKE → Yeşil chase (koşan ışık) animasyonu
 * 3. İttifak Alanı → İttifak rengi (Kırmızı/Mavi) nefes alma (breathe) animasyonu
 * 4. Orta Alan → Sarı dalga (wave) animasyonu
 * 5. IDLE → Yavaş gökkuşağı (rainbow) animasyonu
 */
public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final Supplier<Pose2d> poseSupplier;

    private static final int TOTAL_LENGTH = RobotMap.kLEDLength;

    // --- Robot Durumları ---
    public enum LEDState {
        IDLE,
        ALLIANCE_ZONE,
        NEUTRAL_ZONE,
        INTAKE,
        SHOOTING
    }

    private LEDState currentState = LEDState.IDLE;
    private boolean isShooting = false;
    private boolean isIntaking = false;

    // --- Animasyon Sayaçları ---
    private int rainbowHue = 0;
    private int chaseIndex = 0;
    private int strobeCounter = 0;
    private int frameCounter = 0;

    public LEDSubsystem(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        led = new AddressableLED(RobotMap.kLEDPort);
        buffer = new AddressableLEDBuffer(TOTAL_LENGTH);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();

        System.out.println("[LED] Başlatıldı — " + TOTAL_LENGTH + " LED");
    }

    // --- Dış Durum Ayarlayıcılar ---

    public void setShooting(boolean shooting) {
        this.isShooting = shooting;
    }

    public void setIntaking(boolean intaking) {
        this.isIntaking = intaking;
    }

    // --- Periyodik Güncelleme ---

    @Override
    public void periodic() {
        frameCounter++;

        // Durumu belirle (öncelik sırasıyla)
        currentState = determineState();

        // Animasyonu çalıştır
        switch (currentState) {
            case SHOOTING:
                animateShooting();
                break;
            case INTAKE:
                animateIntake();
                break;
            case ALLIANCE_ZONE:
                animateAllianceBreath();
                break;
            case NEUTRAL_ZONE:
                animateNeutralWave();
                break;
            case IDLE:
            default:
                animateIdleRainbow();
                break;
        }

        led.setData(buffer);
    }

    /**
     * Robot durumunu belirle (öncelik sırasına göre).
     */
    private LEDState determineState() {
        if (isShooting) {
            return LEDState.SHOOTING;
        }
        if (isIntaking) {
            return LEDState.INTAKE;
        }

        // Alan tespiti
        Pose2d pose = poseSupplier.get();
        if (pose != null) {
            double x = pose.getX();
            var alliance = DriverStation.getAlliance();

            if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Red) {
                    if (x >= FieldConstants.kRedAllianceZoneMinX) {
                        return LEDState.ALLIANCE_ZONE;
                    }
                } else {
                    if (x <= FieldConstants.kBlueAllianceZoneMaxX) {
                        return LEDState.ALLIANCE_ZONE;
                    }
                }
                // İttifak alanı dışında → orta alan
                return LEDState.NEUTRAL_ZONE;
            }
        }

        return LEDState.IDLE;
    }

    // =====================================================================
    // ANİMASYONLAR
    // =====================================================================

    /**
     * SHOOTING: Hızlı beyaz strobe/flash efekti.
     * Her 3 frame'de bir beyaz ↔ kapalı.
     */
    private void animateShooting() {
        strobeCounter++;
        boolean on = (strobeCounter / 3) % 2 == 0;

        for (int i = 0; i < buffer.getLength(); i++) {
            if (on) {
                buffer.setRGB(i, 255, 255, 255); // Parlak beyaz
            } else {
                buffer.setRGB(i, 0, 0, 0); // Kapalı
            }
        }
    }

    /**
     * INTAKE: Yeşil chase (koşan ışık) animasyonu.
     * 3 LED genişliğinde yeşil grup şerit boyunca koşar.
     */
    private void animateIntake() {
        if (frameCounter % 3 == 0) {
            chaseIndex = (chaseIndex + 1) % buffer.getLength();
        }

        for (int i = 0; i < buffer.getLength(); i++) {
            // Chase grubunun mesafesine göre parlaklık
            int dist = Math.min(
                    Math.abs(i - chaseIndex),
                    buffer.getLength() - Math.abs(i - chaseIndex));

            if (dist <= 1) {
                buffer.setRGB(i, 0, 255, 0); // Parlak yeşil
            } else if (dist <= 3) {
                int fade = (int) (255.0 * (1.0 - (dist - 1) / 3.0));
                buffer.setRGB(i, 0, Math.max(fade, 0), 0);
            } else {
                buffer.setRGB(i, 0, 15, 0); // Çok hafif yeşil arka plan
            }
        }
    }

    /**
     * İTTİFAK ALANI: İttifak rengi nefes alma (breathing/pulse) animasyonu.
     * Parlaklık sinüs şeklinde artar ve azalır.
     */
    private void animateAllianceBreath() {
        double time = Timer.getFPGATimestamp();
        double breath = (Math.sin(time * 3.0) + 1.0) / 2.0; // 0..1 arasında
        int brightness = (int) (40 + breath * 215); // 40..255 arasında

        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

        for (int i = 0; i < buffer.getLength(); i++) {
            if (isRed) {
                buffer.setRGB(i, brightness, 0, 0);
            } else {
                buffer.setRGB(i, 0, 0, brightness);
            }
        }
    }

    /**
     * ORTA ALAN: Sarı dalga (wave) animasyonu.
     * Sinüs dalgası şerit boyunca ilerler.
     */
    private void animateNeutralWave() {
        double time = Timer.getFPGATimestamp();

        for (int i = 0; i < buffer.getLength(); i++) {
            double wave = (Math.sin(time * 4.0 + i * 0.8) + 1.0) / 2.0;
            int r = (int) (40 + wave * 215);
            int g = (int) (30 + wave * 160);
            buffer.setRGB(i, r, g, 0); // Sarı tonları
        }
    }

    /**
     * IDLE: Yavaş gökkuşağı (rainbow) animasyonu.
     * Tüm renk spektrumu şerit boyunca kayar.
     */
    private void animateIdleRainbow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (rainbowHue + (i * 180 / Math.max(buffer.getLength(), 1))) % 180;
            buffer.setHSV(i, hue, 255, 100); // Düşük parlaklıkta gökkuşağı
        }
        rainbowHue = (rainbowHue + 2) % 180;
    }

    // =====================================================================
    // YARDIMCI METODLAR
    // =====================================================================

    /** Tüm LED'leri belirli bir renge boya */
    public void setAll(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        led.setData(buffer);
    }

    /** LED'leri kapat */
    public void off() {
        setAll(0, 0, 0);
    }

    /** Mevcut LED durumunu döndür */
    public LEDState getCurrentState() {
        return currentState;
    }
}
