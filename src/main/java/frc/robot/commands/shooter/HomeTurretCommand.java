package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Command to HOME the Turret using a magnetic limit switch.
 * Taret 0 noktasında olduğu ve -200/+200 dönebildiği için önce sola 100 derece,
 * bulamazsa sağa 100 derece tarama yapar.
 */
public class HomeTurretCommand extends Command {
    private final ShooterSubsystem shooter;
    private double startPos;
    private int state; // 0: sola dön, 1: sağa dön, 2: bulunamadı

    public HomeTurretCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        startPos = shooter.getTurretEncoderPosition();
        state = 0;
        System.out.println("[HomeTurret] Starting Homing...");
    }

    @Override
    public void execute() {
        if (shooter.isTurretAtHomeLimit()) {
            return; // isFinished() yakalayacak
        }

        double currentPos = shooter.getTurretEncoderPosition();

        if (state == 0) {
            shooter.setTurretPercent(-0.06); // Sola yavaşça dön
            if (currentPos <= startPos - 100.0) {
                state = 1; // 100 derece sola gittik bulamadık, sağa dön
            }
        } else if (state == 1) {
            shooter.setTurretPercent(0.06); // Sağa dön
            if (currentPos >= startPos + 100.0) {
                state = 2; // Başlangıçtan sağa da 100 derece gittik bulamadık, vazgeç
            }
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.isTurretAtHomeLimit() || state == 2;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTurretPercent(0);
        if (shooter.isTurretAtHomeLimit()) {
            shooter.resetTurretEncoder();
            shooter.setTurretAngle(0.0);
            System.out.println("[HomeTurret] Turret Homed to Magnet Switch! Encoder Reset to 0.");
        } else if (!interrupted) {
            System.out.println("[HomeTurret] Turret Homing Failed (Magnet switch not found)!");
        }
    }
}
