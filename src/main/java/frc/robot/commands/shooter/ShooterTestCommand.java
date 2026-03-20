package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Test modunda Shooter PID ayarlaması için komut.
 * Flywheel, Turret ve Hood için canlı tuning yapılmasını sağlar.
 * Auto-aim özelliği dahil.
 * 
 * <h2>Dashboard Erişimi:</h2>
 * <ul>
 * <li><code>/Tuning/ShooterTest/Flywheel RPM</code></li>
 * <li><code>/Tuning/ShooterTest/Turret Angle</code></li>
 * <li><code>/Tuning/ShooterTest/Hood Angle</code></li>
 * <li><code>/Tuning/ShooterTest/Auto Aim</code> (boolean toggle)</li>
 * </ul>
 * 
 * <p>
 * Not: Feeder tuning artık ayrı FeederTestCommand tarafından yönetilir.
 * </p>
 */
public class ShooterTestCommand extends Command {

    private final ShooterSubsystem shooter;

    // Test mode için hedef değerler (Dashboard'dan ayarlanabilir)
    private final TunableNumber testFlywheelRPM;
    private final TunableNumber testTurretAngle;
    private final TunableNumber testHoodAngle;

    public ShooterTestCommand(ShooterSubsystem shooter, frc.robot.subsystems.feeder.FeederSubsystem feeder) {
        this.shooter = shooter;
        // Not: feeder parametresi geriye uyumluluk için korunuyor ama artık
        // kullanılmıyor.
        // Feeder tuning FeederTestCommand tarafından yapılır.
        addRequirements(shooter);

        // TunableNumber'ları constructor'da oluştur
        testFlywheelRPM = new TunableNumber("Shooter/Test", "Flywheel RPM", 0.0);
        testTurretAngle = new TunableNumber("Shooter/Test", "Turret Angle", 0.0);
        testHoodAngle = new TunableNumber("Shooter/Test", "Hood Angle", 40.0);
    }

    @Override
    public void initialize() {
        System.out.println("========================================");
        System.out.println("[ShooterTest] Test modu başlatıldı!");
        System.out.println("[ShooterTest] Dashboard: /Tuning/ShooterTest/");
        System.out.println("[ShooterTest] Auto-aim: SmartDashboard/Tuning/Shooter/EnableAutoAim");
        System.out.println("========================================");
    }

    @Override
    public void execute() {
        // =========== AUTO-AIM CHECK ===========
        boolean autoAimEnabled = shooter.isAutoAimActive();

        if (autoAimEnabled) {
            // Auto-aim modunda - shooter kendi hesaplar (periodic içinde)

        } else {
            // Manuel mod - dashboard değerlerini kullan
            if (shooter.isAutoAimActive()) {
                shooter.disableAutoAim();
            }

            // =========== FLYWHEEL ===========
            double targetRPM = testFlywheelRPM.get();
            if (targetRPM > 0) {
                shooter.setFlywheelRPM(targetRPM);
            } else {
                shooter.stopFlywheel();
            }

            // =========== TURRET ===========
            shooter.setTurretAngleTest(testTurretAngle.get());

            // =========== HOOD ===========
            shooter.setHoodAngleTest(testHoodAngle.get());
        }

        // =========== POV CONTROLS (Manual Nudge or Offset) ===========
        int pov = edu.wpi.first.wpilibj.DriverStation.getStickPOV(1, 0); // Operator Controller, POV 0

        if (pov == 270) { // LEFT
            if (autoAimEnabled) {
                shooter.adjustAutoAimOffset(-0.5);
            } else {
                double current = testTurretAngle.get();
                testTurretAngle.set(current + 1.0);
            }
        } else if (pov == 90) { // RIGHT
            if (autoAimEnabled) {
                shooter.adjustAutoAimOffset(0.5);
            } else {
                double current = testTurretAngle.get();
                testTurretAngle.set(current - 1.0);
            }
        }

        // Test mode log
        if (Constants.tuningMode) {
            Logger.recordOutput("Tuning/Shooter/Test/TargetRPM", autoAimEnabled ? shooter.getFlywheelOffset() : testFlywheelRPM.get());
            Logger.recordOutput("Tuning/Shooter/Test/TargetTurret", autoAimEnabled ? shooter.getAutoAimOffset() : testTurretAngle.get());
            Logger.recordOutput("Tuning/Shooter/Test/TargetHood", autoAimEnabled ? shooter.getHoodOffset() : testHoodAngle.get());
        }
        Logger.recordOutput("Tuning/Shooter/Test/Mode", autoAimEnabled ? "AUTO-AIM" : "MANUAL");
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disableAutoAim();
        shooter.stopAll();
        System.out.println("[ShooterTest] Test modu sonlandırıldı");
    }

    @Override
    public boolean isFinished() {
        return false; // Test modunda sürekli çalışır
    }
}
