package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Shooter Sıfırlama Komutu (Shoot Reset).
 * Shooter Reset Command.
 *
 * <h3>Ne zaman kullanılır? / When to use?</h3>
 * <p>
 * Eğer taret veya hood açıları yanlış kalibrasyon nedeniyle bozulduysa,
 * robot DISABLED konumundayken:
 * </p>
 * <ol>
 *   <li>Tureti elle 0° (düz ileri) konumuna getir / Manually turn turret to 0° (straight forward)</li>
 *   <li>Hood'u elle 0° (tam kapalı / flat) konumuna bas / Manually press hood to 0° (fully closed/flat)</li>
 *   <li>Dashboard'dan bu komutu çalıştır / Run this command from Dashboard</li>
 *   <li>Hem turret hem hood encoder'ı şu anki fiziksel konuma (0°) sıfırlar /
 *       Both turret and hood encoders are reset to current physical position (0°)</li>
 * </ol>
 *
 * <p><b>NOT:</b> Bu komut motorlara herhangi bir güç uygulamaz. Sadece encoder değerlerini
 * sıfırlar. Robot disabled konumundayken bile çalışır (ignoringDisable).</p>
 *
 * <p><b>NOTE:</b> This command applies no power to motors. It only resets encoder values.
 * Works even while robot is disabled (ignoringDisable).</p>
 */
public class ShootResetCommand extends Command {

    private final ShooterSubsystem shooter;

    /**
     * @param shooter ShooterSubsystem — turret ve hood enkoderleri sıfırlanacak /
     *                turret and hood encoders will be reset
     */
    public ShootResetCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        // NOT: addRequirements KASITLI olarak çağrılmıyor!
        // NOTE: addRequirements is INTENTIONALLY NOT called!
        // Bu komut sadece encoder sıfırlar, motor talep etmiyor.
        // This command only resets encoders, does not claim motor control.
        // ignoringDisable() ile birlikte disabled konumunda da çalışabilmesi için.
        // Combined with ignoringDisable(), so it works while disabled.
    }

    @Override
    public void initialize() {
        // Turret ve hood enkoderi sıfırla
        // Reset turret and hood encoders
        shooter.resetTurretEncoder();
        shooter.resetHoodEncoder();

        System.out.println("[ShootReset] ✅ Turret + Hood encoder'lar sıfırlandı!");
        System.out.println("[ShootReset] ✅ Turret + Hood encoders reset!");
        System.out.println("[ShootReset] ⚠️ Robot artık taret=0° ve hood=0° sanıyor.");
        System.out.println("[ShootReset] ⚠️ Robot now assumes turret=0° and hood=0°.");
    }

    @Override
    public boolean isFinished() {
        // Tek seferlik komut — initialize'da her şey yapılır
        // One-shot command — everything done in initialize
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        // Disabled konumunda da çalışabilmeli
        // Must work while robot is disabled
        return true;
    }
}
