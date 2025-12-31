package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Tam atış sekansı:
 * 1. Flywheel'i hızlandır
 * 2. Taret hedefe kilitlenene kadar bekle
 * 3. Intake/Feeder ile besle
 * 
 * NOT: Cleanup (stopShooter, runRoller(0)) RobotContainer'da onFalse ile
 * yapılır.
 */
public class ShootCommand extends SequentialCommandGroup {

    /**
     * @param shooterSubsystem Shooter alt sistemi
     * @param intakeSubsystem  Intake alt sistemi (besleme için)
     */
    public ShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                // 1. Flywheel'i başlat
                Commands.runOnce(() -> shooterSubsystem.shoot()),

                // 2. Taret hedefe kilitlenene kadar bekle
                Commands.waitUntil(() -> shooterSubsystem.isAimingAtTarget()),

                // 3. Intake motorunu çalıştır (mermiyi it)
                Commands.run(() -> intakeSubsystem.runRoller(12.0), intakeSubsystem));

        addRequirements(shooterSubsystem);
    }
}
