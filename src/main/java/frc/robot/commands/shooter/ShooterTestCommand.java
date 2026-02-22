package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
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
 */
public class ShooterTestCommand extends Command {

    private final ShooterSubsystem shooter;
    private final frc.robot.subsystems.feeder.FeederSubsystem feeder;

    // Logic State
    private boolean isFeeding = false;

    // Test mode için hedef değerler (Dashboard'dan ayarlanabilir)
    private final TunableNumber testFlywheelRPM;
    private final TunableNumber testTurretAngle;
    private final TunableNumber testHoodAngle;

    public ShooterTestCommand(ShooterSubsystem shooter, frc.robot.subsystems.feeder.FeederSubsystem feeder) {
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(shooter, feeder);

        // TunableNumber'ları constructor'da oluştur
        testFlywheelRPM = new TunableNumber("Shooter/Test", "Flywheel RPM", 0.0);
        testTurretAngle = new TunableNumber("Shooter/Test", "Turret Angle", 0.0);
        testHoodAngle = new TunableNumber("Shooter/Test", "Hood Angle", 40.0);

        // Auto-aim toggle için SmartDashboard
        // Auto-aim toggle için SmartDashboard -> "Shooter/EnableAutoAim" (Subsystem'de)
    }

    @Override
    public void initialize() {
        System.out.println("========================================");
        System.out.println("[ShooterTest] Test modu başlatıldı!");
        System.out.println("[ShooterTest] Dashboard: /Tuning/ShooterTest/");
        System.out.println("[ShooterTest] Dashboard: /Tuning/ShooterTest/");
        System.out.println("[ShooterTest] Auto-aim: SmartDashboard/Shooter/EnableAutoAim");
        System.out.println("========================================");
    }

    @Override
    public void execute() {
        // =========== AUTO-AIM CHECK ===========
        // Subsystem state is the source of truth (controlled by Dashboard or
        // Controller)
        boolean autoAimEnabled = shooter.isAutoAimActive();

        if (autoAimEnabled) {
            // Auto-aim modunda - shooter kendi hesaplar (periodic içinde)
            // manual setpoint göndermiyoruz.

        } else {
            // Manuel mod - dashboard değerlerini kullan
            // isAutoAimActive kontrolü subsystemde zaten yapılıyor ama burada manual
            // setpoint göndermeden önce emin olmak iyidir
            if (shooter.isAutoAimActive()) {
                shooter.disableAutoAim();
            }

            // =========== FLYWHEEL ===========
            double targetRPM = testFlywheelRPM.get();
            if (targetRPM > 0) {
                shooter.setFlywheelRPM(targetRPM);

                // --- AUTO FEED LOGIC ---
                // Flywheel hedefe ulaştığında feeder'ı başlat ve KİLİTLE (Latch).
                // RPM düşse bile feeder çalışmaya devam etsin.
                if (shooter.isFlywheelAtTarget()) {
                    isFeeding = true;
                }
            } else {
                shooter.stopFlywheel();
                isFeeding = false; // Reset latch
            }

            // Feeder Logic
            if (isFeeding) {
                feeder.feed(); // Velocity Control
            } else {
                feeder.stop();
            }

            // =========== TURRET ===========
            shooter.setTurretAngleTest(testTurretAngle.get());

            // =========== HOOD ===========
            shooter.setHoodAngleTest(testHoodAngle.get());
        }

        // =========== POV CONTROLS (Manual Nudge or Offset) ===========
        // Not: ControllerBindings test modunda da çalışabilir,
        // ancak burada özel mantık ekliyoruz.
        int pov = edu.wpi.first.wpilibj.DriverStation.getStickPOV(1, 0); // Operator Controller, POV 0

        if (pov == 270) { // LEFT
            if (autoAimEnabled) {
                shooter.adjustAutoAimOffset(-0.5); // Sadece offset
            } else {
                // Manual Nudge
                double current = testTurretAngle.get();
                testTurretAngle.set(current + 1.0); // Sola dön (derece artar)
            }
        } else if (pov == 90) { // RIGHT
            if (autoAimEnabled) {
                shooter.adjustAutoAimOffset(0.5); // Sadece offset
            } else {
                // Manual Nudge
                double current = testTurretAngle.get();
                testTurretAngle.set(current - 1.0); // Sağa dön (derece azalır)
            }
        }

        // Test mode log (diğer değerler subsystem.periodic() de loglanıyor)
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
