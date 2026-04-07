package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
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
        private final CommandXboxController operatorController;

        private final DriveSubsystem driveSubsystem;
        private final VisionSubsystem visionSubsystem;
        private final ShooterSubsystem shooterSubsystem;
        private final FeederSubsystem feederSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final LEDSubsystem ledSubsystem;
        private final DigitalInput mz80_8;
        private final DigitalInput mz80_9;


        // Intake toggle durumu
        private boolean intakeRunning = false;

        public ControllerBindings(
                        CommandXboxController driverController,
                        CommandXboxController operatorController,
                        DriveSubsystem driveSubsystem,
                        VisionSubsystem visionSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        FeederSubsystem feederSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LEDSubsystem ledSubsystem,
                        DigitalInput mz80_8,
                        DigitalInput mz80_9) {

                this.driverController = driverController;
                this.operatorController = operatorController;
                this.driveSubsystem = driveSubsystem;
                this.visionSubsystem = visionSubsystem;
                this.shooterSubsystem = shooterSubsystem;
                this.feederSubsystem = feederSubsystem;
                this.intakeSubsystem = intakeSubsystem;
                this.ledSubsystem = ledSubsystem;
                this.mz80_8 = mz80_8;
                this.mz80_9 = mz80_9;
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
                                .whileTrue(new frc.robot.commands.drive.TrenchPassCommand(driveSubsystem,
                                                shooterSubsystem));

                // [LEFT BUMPER] İTTİFAKA VE KONUMA GÖRE GÜVENLİ BUMP PASS
                driverController.leftBumper()
                                .whileTrue(new frc.robot.commands.drive.BumpPassCommand(driveSubsystem,
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
                                new frc.robot.commands.shooter.ShootCommand(
                                                shooterSubsystem,
                                                feederSubsystem,
                                                driveSubsystem,
                                                intakeSubsystem,
                                                mz80_8,
                                                mz80_9,
                                                driveSubsystem::getPose));

                // [SOL TETİK] INTAKE ROLLER TOGGLE
                // İlk basışta intake çalışır, tekrar basınca durur
                driverController.leftTrigger().onTrue(
                                Commands.runOnce(() -> {
                                        intakeRunning = !intakeRunning;
                                        if (intakeRunning) {
                                                intakeSubsystem.deploy();
                                                intakeSubsystem.runRollerRPM(
                                                                frc.robot.constants.IntakeConstants.kRollerTargetRPM);
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

                // [Y BUTTON] HOOD AÇISINI 0 YAP (KAPANMA)
                // Y tuşuna basıldığında (onTrue), Hood motoru 0 dereceye konumlanır.
                driverController.y().onTrue(
                                Commands.runOnce(() -> shooterSubsystem.setHoodAngle(0), shooterSubsystem));

                // [POV BUTTONS] ROBOT YÖNÜNÜ AYARLA (Orientation Snap)
                // POV Up: 0° (İleri), Right: 270° (Sağ), Down: 180° (Geri), Left: 90° (Sol)
                driverController.povUp().onTrue(Commands.runOnce(
                                () -> driveSubsystem.resetRotation(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)),
                                driveSubsystem));
                driverController.povRight().onTrue(Commands.runOnce(
                                () -> driveSubsystem.resetRotation(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(270)),
                                driveSubsystem));
                driverController.povDown().onTrue(Commands.runOnce(
                                () -> driveSubsystem.resetRotation(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180)),
                                driveSubsystem));
                driverController.povLeft().onTrue(Commands.runOnce(
                                () -> driveSubsystem.resetRotation(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90)),
                                driveSubsystem));
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

                // ==================== HOOD OFFSET (BUMPERS) ====================
                operatorController.leftBumper().onTrue(shooterSubsystem.decreaseHoodOffsetCommand());
                operatorController.rightBumper().onTrue(shooterSubsystem.increaseHoodOffsetCommand());

                // ==================== HUB POZİSYON OFFSET (POV) ====================
                // Hub hedef noktasını kaydırarak RPM/açı hesaplamasını ince ayar yapar
                // POV Up/Down: Y offset (±0.1 metre)
                operatorController.povUp().onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHubOffset(0.0, 0.1)));
                operatorController.povDown()
                                .onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHubOffset(0.0, -0.1)));

                // POV Left/Right: X offset (±0.1 metre)
                operatorController.povLeft()
                                .onTrue(Commands.runOnce(() -> shooterSubsystem.decreaseHoodOffsetCommand()));
                operatorController.povRight()
                                .onTrue(Commands.runOnce(() -> shooterSubsystem.increaseHoodOffsetCommand()));
                // ============================================================
                                // [RIGHT BUMPER] İTTİFAKA VE KONUMA GÖRE A->B veya B->A SIRALI GİDİŞ
                operatorController.rightTrigger(0.8)
                                .whileTrue(new frc.robot.commands.drive.TrenchPassCommand(driveSubsystem,
                                                shooterSubsystem));

                // [LEFT TRIGGER] BumpPass kaldırıldı — L2 sadece intake için kullanılıyor
        }

        /**
         * Alliance'a göre ideal atış pozisyonunu hesaplar.
         */
        private Pose2d getShootingPose() {
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
