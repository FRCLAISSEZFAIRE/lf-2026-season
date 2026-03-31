package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;

import frc.robot.commands.intake.IntakeCollectCommand;
import frc.robot.commands.drive.TrenchPassCommand;
import frc.robot.commands.drive.BumpPassCommand;
import frc.robot.commands.shooter.ShootCommand;
/**
 * Controller binding'lerini yöneten sınıf.
 * Tek joystick (driver) kullanılır.
 */
public class ControllerBindings {

        private final CommandXboxController driverController;
        private final CommandXboxController operatorController;

        private final DriveSubsystem driveSubsystem;
        private final VisionSubsystem visionSubsystem;
        private final ShooterSubsystem shooterSubsystem;
        private final FeederSubsystem feederSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final LEDSubsystem ledSubsystem;


        public ControllerBindings(
                        CommandXboxController driverController,
                        CommandXboxController operatorController,
                        DriveSubsystem driveSubsystem,
                        VisionSubsystem visionSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        FeederSubsystem feederSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LEDSubsystem ledSubsystem) {

                this.driverController = driverController;
                this.operatorController = operatorController;
                this.driveSubsystem = driveSubsystem;
                this.visionSubsystem = visionSubsystem;
                this.shooterSubsystem = shooterSubsystem;
                this.feederSubsystem = feederSubsystem;
                this.intakeSubsystem = intakeSubsystem;
                this.ledSubsystem = ledSubsystem;
        }

        /** Tüm binding'leri yapılandır */
        public void configureAll() {
                configureDriverBindings();
                configureOperatorBindings();
        }

        // =========================================================================
        // DRIVER BINDINGS (Tek Joystick)
        // =========================================================================
        private void configureDriverBindings() {

                // [RIGHT BUMPER] İTTİFAKA VE KONUMA GÖRE A->B veya B->A SIRALI GİDİŞ
                driverController.rightBumper()
                                .whileTrue(new TrenchPassCommand(driveSubsystem,
                                                shooterSubsystem));

                // [LEFT BUMPER] İTTİFAKA VE KONUMA GÖRE GÜVENLİ BUMP PASS
                driverController.leftBumper()
                                .whileTrue(new BumpPassCommand(driveSubsystem,
                                                shooterSubsystem));

                // [A BUTTON] INTAKE DEPLOY
                driverController.a()
                                .onTrue(Commands.runOnce(intakeSubsystem::deploy, intakeSubsystem));

                // [B BUTTON] INTAKE RETRACT
                driverController.b()
                                .onTrue(Commands.runOnce(intakeSubsystem::retract, intakeSubsystem));

                // ============================================================

                // [SAĞ TETİK] POSE TABANLI ATIŞ (Akıllı Atış)
                // Robot pozisyonuna göre RPM, Hood, Taret hesaplar.
                // Hazır olduğunda ateşler, kilitlenir.
                // Şasi SERBEST (Manual sürüş devam eder). Taret otomatik hedefler.
                driverController.rightTrigger().whileTrue(
                                Commands.parallel(
                                                new ShootCommand(
                                                                shooterSubsystem,
                                                                feederSubsystem,
                                                                driveSubsystem,
                                                                intakeSubsystem,
                                                                driveSubsystem::getPose),
                                                Commands.startEnd(
                                                                () -> ledSubsystem.setShooting(true),
                                                                () -> ledSubsystem.setShooting(false))));

                // [SOL TETİK] INTAKE COLLECT (Toggle)
                driverController.leftTrigger().toggleOnTrue(
                                Commands.parallel(
                                                new IntakeCollectCommand(intakeSubsystem, feederSubsystem),
                                                Commands.startEnd(
                                                                () -> ledSubsystem.setIntaking(true),
                                                                () -> ledSubsystem.setIntaking(false))));

                // [X BUTTON] KUSMA (Reverse Intake + Feeder)
                // Basılı tutulduğunda intake ve feeder ters çalışır
                driverController.x().whileTrue(
                                Commands.parallel(
                                                Commands.run(() -> intakeSubsystem.runRollerRPM(-4800),
                                                                intakeSubsystem),
                                                Commands.run(feederSubsystem::reverse, feederSubsystem))
                                                .finallyDo(() -> {
                                                        intakeSubsystem.stopRoller();
                                                        feederSubsystem.stop();
                                                }));

                // [Y BUTTON] HOOD AÇISINI 0 YAP (KAPANMA)
                // Y tuşuna basıldığında (onTrue), Hood motoru 0 dereceye konumlanır.
                driverController.y().onTrue(
                                Commands.runOnce(() -> shooterSubsystem.setHoodAngle(0), shooterSubsystem));

        }

        // =========================================================================
        // OPERATOR BINDINGS (Yardımcı Sürücü Mekanizma Kontrolü)
        // =========================================================================
        private void configureOperatorBindings() {

                // [A BUTTON] INTAKE DEPLOY
                operatorController.a()
                                .onTrue(Commands.runOnce(intakeSubsystem::deploy, intakeSubsystem));

                // [B BUTTON] INTAKE RETRACT
                operatorController.b()
                                .onTrue(Commands.runOnce(intakeSubsystem::retract, intakeSubsystem));

                // [X BUTTON] FLYWHEEL OFFSET -50 RPM
                operatorController.x().onTrue(
                                Commands.runOnce(() -> shooterSubsystem.adjustFlywheelOffset(-50.0), shooterSubsystem)
                                                .ignoringDisable(true));

                // [Y BUTTON] FLYWHEEL OFFSET +50 RPM
                operatorController.y().onTrue(
                                Commands.runOnce(() -> shooterSubsystem.adjustFlywheelOffset(50.0), shooterSubsystem)
                                                .ignoringDisable(true));

                // ==================== HUB POZİSYON OFFSET (POV) ====================
                // Hub hedef noktasını kaydırarak RPM/açı hesaplamasını ince ayar yapar
                // POV Up/Down: Y offset (±0.1 metre)
                operatorController.povUp().onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHubOffset(0.0, 0.1)));
                operatorController.povDown().onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHubOffset(0.0, -0.1)));

                // POV Left/Right: X offset (±0.1 metre)
                operatorController.povLeft().onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHubOffset(-0.1, 0.0)));
                operatorController.povRight().onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHubOffset(0.1, 0.0)));
                // ============================================================
        }

}
