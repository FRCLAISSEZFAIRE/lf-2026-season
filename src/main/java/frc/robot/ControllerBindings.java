package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;

import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.AutoIntakeCommand;
import frc.robot.commands.shooter.ShootCommand;

import frc.robot.commands.climber.ClimberExtendCommand;
import frc.robot.commands.climber.ClimberRetractCommand;

/**
 * Controller binding'lerini yöneten sınıf.
 * 
 * <p>
 * RobotContainer'ı temiz tutmak için tüm buton atamaları bu sınıfta
 * toplanmıştır.
 * </p>
 * 
 * <h2>Controller Dağılımı:</h2>
 * <ul>
 * <li><b>Driver Controller (Port 0):</b> Sürüş, Gyro, X-Stance</li>
 * <li><b>Operator Controller (Port 1):</b> Tüm mekanizmalar</li>
 * </ul>
 * 
 * <h2>Operator Buton Grupları:</h2>
 * <ul>
 * <li><b>Intake/Feeder:</b> Sağ/Sol Tetik</li>
 * <li><b>Shooter:</b> Sağ Bumper</li>
 * <li><b>Climber:</b> Y (Extend), A (Retract), POV (Manuel)</li>
 * <li><b>Acil:</b> Start butonu</li>
 * </ul>
 * 
 * @author FRC Team
 * @see RobotContainer
 */
public class ControllerBindings {

        private final CommandXboxController driverController;
        private final CommandXboxController operatorController;

        // Subsystem referansları
        // Subsystem referansları
        private final DriveSubsystem driveSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final ShooterSubsystem shooterSubsystem;
        private final ClimberSubsystem climberSubsystem;
        private final FeederSubsystem feederSubsystem;
        private final LEDSubsystem ledSubsystem;
        private final frc.robot.subsystems.vision.VisionSubsystem visionSubsystem;
        private final java.util.function.Consumer<Integer> climbPosChanger;

        public ControllerBindings(
                        CommandXboxController driverController,
                        CommandXboxController operatorController,
                        DriveSubsystem driveSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        ClimberSubsystem climberSubsystem,
                        FeederSubsystem feederSubsystem,
                        LEDSubsystem ledSubsystem,
                        frc.robot.subsystems.vision.VisionSubsystem visionSubsystem,
                        java.util.function.Consumer<Integer> climbPosChanger) {

                this.driverController = driverController;
                this.operatorController = operatorController;
                this.driveSubsystem = driveSubsystem;
                this.intakeSubsystem = intakeSubsystem;
                this.shooterSubsystem = shooterSubsystem;
                this.climberSubsystem = climberSubsystem;
                this.feederSubsystem = feederSubsystem;
                this.ledSubsystem = ledSubsystem;
                this.visionSubsystem = visionSubsystem;
                this.climbPosChanger = climbPosChanger;
        }

        /** Tüm binding'leri yapılandır */
        public void configureAll() {
                configureDriverBindings();
                configureOperatorBindings();
        }

        // ==================== DRIVER BINDINGS ====================
        private void configureDriverBindings() {
                // Driver Controller Temizlendi.
                // Yeni bindingler RobotContainer içinde tanımlanıyor:
                // POV: Skor Seçimi
                // X: Pathfinding
        }

        private void configureOperatorBindings() {
                configureIntakeBindings();
                configureShooterBindings();
                configureClimberBindings();
                configureEmergencyBindings();
        }

        // --- INTAKE / FEEDER ---
        private void configureIntakeBindings() {
                // Right Trigger: Intake + Feeder (Normal Alma)
                operatorController.rightTrigger(0.5)
                                .whileTrue(Commands.parallel(
                                                new RunIntakeCommand(intakeSubsystem, 12.0),
                                                Commands.run(() -> feederSubsystem.feed(), feederSubsystem),
                                                Commands.runOnce(() -> ledSubsystem.setIntaking())))
                                .onFalse(Commands.parallel(
                                                Commands.runOnce(feederSubsystem::stop, feederSubsystem),
                                                Commands.runOnce(() -> ledSubsystem.setIdle())));

                // Left Trigger: SADECE Intake Ters (Çıkarma) - Feeder durur
                operatorController.leftTrigger(0.5)
                                .whileTrue(new RunIntakeCommand(intakeSubsystem, -12.0))
                                .onFalse(Commands.runOnce(() -> intakeSubsystem.runRoller(0), intakeSubsystem));

                // X Button: Auto-Intake (Sürücüdeki gibi kamera takibi)
                operatorController.x()
                                .whileTrue(new AutoIntakeCommand(driveSubsystem, intakeSubsystem, feederSubsystem,
                                                visionSubsystem))
                                .onFalse(Commands.runOnce(() -> ledSubsystem.setIdle()));
        }

        // --- SHOOTER ---
        private void configureShooterBindings() {
                // Right Bumper: Atış (Manuel)
                operatorController.rightBumper()
                                .whileTrue(Commands.sequence(
                                                Commands.runOnce(() -> ledSubsystem
                                                                .setState(LEDSubsystem.LEDState.SHOOTING)),
                                                new ShootCommand(shooterSubsystem, feederSubsystem)))
                                .onFalse(Commands.parallel(
                                                Commands.runOnce(shooterSubsystem::stopShooter),
                                                Commands.runOnce(feederSubsystem::stop, feederSubsystem),
                                                Commands.runOnce(() -> ledSubsystem.setIdle())));

                // Left Bumper: Flywheel Ters (Sıkışma Giderme)
                operatorController.leftBumper()
                                .whileTrue(Commands.run(shooterSubsystem::reverse, shooterSubsystem))
                                .onFalse(Commands.runOnce(shooterSubsystem::stopMotorTotal, shooterSubsystem));

                // Y Button: Auto Aim & Shoot
                // Robotu hedefe döndürür (DriveWithAiming) VE Atış yapar (ShootCommand)
                operatorController.y()
                                .whileTrue(Commands.parallel(
                                                new frc.robot.commands.drive.DriveWithAiming(
                                                                driveSubsystem,
                                                                () -> 0.0, // X hızı yok (Sabit)
                                                                () -> 0.0, // Y hızı yok (Sabit)
                                                                () -> new edu.wpi.first.math.geometry.Translation2d(
                                                                                16.5, 5.55) // Hedef
                                                ),
                                                Commands.sequence(
                                                                Commands.runOnce(() -> ledSubsystem.setState(
                                                                                LEDSubsystem.LEDState.SHOOTING)),
                                                                new ShootCommand(shooterSubsystem, feederSubsystem))))
                                .onFalse(Commands.parallel(
                                                Commands.runOnce(driveSubsystem::stop),
                                                Commands.runOnce(shooterSubsystem::stopShooter),
                                                Commands.runOnce(feederSubsystem::stop, feederSubsystem),
                                                Commands.runOnce(() -> ledSubsystem.setIdle())));
        }

        // --- CLIMBER ---
        private void configureClimberBindings() {
                // B Button: Extend (Yukarı)
                operatorController.b()
                                .onTrue(new ClimberExtendCommand(climberSubsystem));

                // A Button: Retract (Aşağı - Tırmanma)
                operatorController.a()
                                .onTrue(new ClimberRetractCommand(climberSubsystem));

                // POV Up: Manuel Yukarı
                operatorController.povUp()
                                .whileTrue(Commands.run(climberSubsystem::manualUp, climberSubsystem))
                                .onFalse(Commands.runOnce(climberSubsystem::stop, climberSubsystem));

                // POV Down: Manuel Aşağı
                operatorController.povDown()
                                .whileTrue(Commands.run(climberSubsystem::manualDown, climberSubsystem))
                                .onFalse(Commands.runOnce(climberSubsystem::stop, climberSubsystem));

                // POV Left: Kule Seçimi (Önceki)
                operatorController.povLeft()
                                .onTrue(Commands.runOnce(() -> climbPosChanger.accept(-1)));

                // POV Right: Kule Seçimi (Sonraki)
                operatorController.povRight()
                                .onTrue(Commands.runOnce(() -> climbPosChanger.accept(1)));
        }

        // --- EMERGENCY ---
        private void configureEmergencyBindings() {

                // Start: Tüm mekanizmaları sıfırla (acil)
                operatorController.start()
                                .onTrue(Commands.parallel(
                                                Commands.runOnce(climberSubsystem::stop),
                                                Commands.runOnce(feederSubsystem::stop),
                                                Commands.runOnce(shooterSubsystem::stopShooter),
                                                Commands.runOnce(() -> ledSubsystem
                                                                .setState(LEDSubsystem.LEDState.ERROR))));
        }
}
