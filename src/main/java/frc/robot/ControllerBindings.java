package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Controller binding'lerini yöneten sınıf.
 * Tek joystick (driver) kullanılır.
 */
public class ControllerBindings {

        private final CommandXboxController driverController;

        private final DriveSubsystem driveSubsystem;
        private final VisionSubsystem visionSubsystem;
        private final ShooterSubsystem shooterSubsystem;
        private final FeederSubsystem feederSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final LEDSubsystem ledSubsystem;

        // Intake toggle durumu
        private boolean intakeRunning = false;

        public ControllerBindings(
                        CommandXboxController driverController,
                        DriveSubsystem driveSubsystem,
                        VisionSubsystem visionSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        FeederSubsystem feederSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LEDSubsystem ledSubsystem) {

                this.driverController = driverController;
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
        }

        // =========================================================================
        // DRIVER BINDINGS (Tek Joystick)
        // =========================================================================
        private void configureDriverBindings() {

                // [RIGHT BUMPER] İTTİFAKA VE KONUMA GÖRE A->B veya B->A SIRALI GİDİŞ
                driverController.rightBumper()
                                .whileTrue(new frc.robot.commands.drive.TrenchPassCommand(driveSubsystem,
                                                shooterSubsystem));

                // [LEFT BUMPER] İTTİFAKA VE KONUMA GÖRE GÜVENLİ BUMP PASS
                driverController.leftBumper()
                                .whileTrue(new frc.robot.commands.drive.BumpPassCommand(driveSubsystem,
                                                shooterSubsystem));

                // [A BUTTON] HOOD AÇISINI 0 YAP (KAPANMA)
                driverController.a()
                                .onTrue(Commands.runOnce(() -> shooterSubsystem.setHoodAngle(0), shooterSubsystem));

                // [B BUTTON] OTOMATİK FEED PASS (Top Toplama Rotası)
                driverController.b()
                                .whileTrue(new frc.robot.commands.intake.FeedPassCommand(driveSubsystem,
                                                intakeSubsystem));

                // ==================== HUB POZİSYON OFFSET (POV) ====================
                // Hub hedef noktasını kaydırarak RPM/açı hesaplamasını ince ayar yapar
                // POV Up/Down: Y offset (±0.1 metre)
                driverController.povUp().onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHubOffset(0.0, 0.1)));
                driverController.povDown().onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHubOffset(0.0, -0.1)));

                // POV Left/Right: X offset (±0.1 metre)
                driverController.povLeft().onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHubOffset(-0.1, 0.0)));
                driverController.povRight().onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHubOffset(0.1, 0.0)));
                // ============================================================

                // [SAĞ TETİK] POSE TABANLI ATIŞ (Akıllı Atış)
                // Robot pozisyonuna göre RPM, Hood, Taret hesaplar.
                // Hazır olduğunda ateşler, kilitlenir.
                // Şasi SERBEST (Manual sürüş devam eder). Taret otomatik hedefler.
                driverController.rightTrigger().whileTrue(
                                new frc.robot.commands.shooter.ShootCommand(
                                                shooterSubsystem,
                                                feederSubsystem,
                                                driveSubsystem,
                                                intakeSubsystem,
                                                driveSubsystem::getPose));

                // [SOL TETİK] INTAKE ROLLER TOGGLE
                // İlk basışta intake çalışır, tekrar basınca durur
                driverController.leftTrigger().onTrue(
                                Commands.runOnce(() -> {
                                        intakeRunning = !intakeRunning;
                                        if (intakeRunning) {
                                                intakeSubsystem.runRollerRPM(4800);
                                        } else {
                                                intakeSubsystem.stopRoller();
                                        }
                                }, intakeSubsystem));

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

                // [Y BUTTON] TOWER'A OTOMATIK SÜRÜŞ
                // Basılı tutulduğu sürece alliance'a göre atış pozisyonuna gider
                driverController.y().whileTrue(
                                new frc.robot.commands.drive.SimpleDriveToPose(
                                                driveSubsystem,
                                                getShootingPose()));

        }

        /**
         * Alliance'a göre ideal atış pozisyonunu hesaplar.
         */
        private edu.wpi.first.math.geometry.Pose2d getShootingPose() {
                var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
                edu.wpi.first.math.geometry.Translation2d hubCenter = frc.robot.constants.FieldConstants
                                .getHubCenter(alliance);

                if (alliance.isPresent()
                                && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                        return new edu.wpi.first.math.geometry.Pose2d(
                                        hubCenter.getX()
                                                        - frc.robot.constants.FieldConstants.kIdealShootingDistanceMeters,
                                        hubCenter.getY(),
                                        edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));
                } else {
                        return new edu.wpi.first.math.geometry.Pose2d(
                                        hubCenter.getX()
                                                        + frc.robot.constants.FieldConstants.kIdealShootingDistanceMeters,
                                        hubCenter.getY(),
                                        edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
                }
        }
}
