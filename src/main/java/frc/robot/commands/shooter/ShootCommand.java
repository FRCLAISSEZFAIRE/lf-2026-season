package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;

/**
 * Otomatik Atış Komutu (Fuel Tank Logic):
 * 1. Flywheel'i başlat.
 * 2. Hedefe kilitlen.
 * 3. Yakıt bitene kadar (veya manuel durdurulana kadar) Feeder'ı çalıştır.
 * 4. Yakıt bitince her şeyi durdur.
 */
public class ShootCommand extends SequentialCommandGroup {

    /**
     * @param shooterSubsystem Shooter alt sistemi
     * @param feederSubsystem  Feeder alt sistemi (Yakıt tankı kontrolü)
     */
    public ShootCommand(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
        
        addCommands(
                // 1. Flywheel'i başlat (Hızlanma)
                Commands.runOnce(() -> shooterSubsystem.shoot(), shooterSubsystem),

                // 2. Taret hedefe kilitlenene kadar bekle (Güvenli atış)
                Commands.waitUntil(() -> shooterSubsystem.isAimingAtTarget()),

                // 3. Yakıt bitene kadar besle (Yükleme modu hariç)
                Commands.run(() -> {
                    if (!feederSubsystem.isLoading()) {
                        feederSubsystem.feed();
                    } else {
                        feederSubsystem.stop();
                    }
                }, feederSubsystem)
                .until(() -> feederSubsystem.isFuelSystemEmpty() && !feederSubsystem.isLoading())
                
                // 4. Bitiş: Durdur
                .andThen(Commands.runOnce(() -> {
                    shooterSubsystem.stopShooter();
                    feederSubsystem.stop();
                }, shooterSubsystem, feederSubsystem))
        );

        addRequirements(shooterSubsystem, feederSubsystem);
    }
}
