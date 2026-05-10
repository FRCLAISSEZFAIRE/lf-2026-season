package frc.robot;

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

                // [Y BUTTON] MANUEL YAVAŞ SÜRÜŞ BİLGİSİ (TOGGLE)
                // Y tuşuna basılması, "shoot" hız çarpanını manuel olarak açıp kapatır (Slow
                // [POV BUTTONS] ROBOT YÖNÜNÜ AYARLA (Orientation Snap)
                // POV Up: 0° (İleri), Right: 270° (Sağ), Down: 180° (Geri), Left: 90° (Sol)

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
                                Commands.runOnce(() -> shooterSubsystem.adjustFlywheelOffset(-50.0))
                                                .ignoringDisable(true));

                // [Y BUTTON] FLYWHEEL OFFSET +50 RPM
                operatorController.y().onTrue(
                                Commands.runOnce(() -> shooterSubsystem.adjustFlywheelOffset(50.0))
                                                .ignoringDisable(true));

                // ==================== TURRET OFFSET (BUMPERS) ====================
                operatorController.leftBumper()
                                .onTrue(Commands.runOnce(() -> shooterSubsystem.adjustAutoAimOffset(-1.0))
                                                .ignoringDisable(true).withName("TurretOffset -1"));
                operatorController.rightBumper()
                                .onTrue(Commands.runOnce(() -> shooterSubsystem.adjustAutoAimOffset(1.0))
                                                .ignoringDisable(true).withName("TurretOffset +1"));

                // ==================== HOOD OFFSET (D-PAD UP/DOWN) ====================
                operatorController.povUp()
                                .onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHoodOffset(1.0))
                                                .ignoringDisable(true).withName("HoodOffset +1"));
                operatorController.povDown()
                                .onTrue(Commands.runOnce(() -> shooterSubsystem.adjustHoodOffset(-1.0))
                                                .ignoringDisable(true).withName("HoodOffset -1"));

                // ==================== RESET GYRO (RIGHT TRIGGER) ====================
                operatorController.rightTrigger()
                                .onTrue(Commands.runOnce(() -> {
                                        driveSubsystem.zeroHeading();
                                        System.out.println("[Operator] Gyro sıfırlandı!");
                                }).ignoringDisable(true).withName("ResetGyro"));

                // ==================== ROBOT POSE OFFSET (LEFT JOYSTICK) ====================
                // Robotun odometri pozisyonunu sürekli olarak kaydırır (sürekli eğimli - analog
                // kontrol).
                // Bu özellik Limelight veya odometri hatasını düzeltmek için robotun sanal
                // pozisyonunu sürükler.
                new edu.wpi.first.wpilibj2.command.button.Trigger(() -> Math.abs(operatorController.getLeftX()) > 0.1
                                || Math.abs(operatorController.getLeftY()) > 0.1).whileTrue(
                                                Commands.run(() -> {
                                                        // X ekseni (sol/sağ) robotun Y koordinatını (yanal) değiştirir
                                                        // (Field referanslı)
                                                        double joyX = edu.wpi.first.math.MathUtil.applyDeadband(
                                                                        operatorController.getLeftX(), 0.1);
                                                        // Y ekseni (ileri/geri) robotun X koordinatını (ileri)
                                                        // değiştirir. Xbox'ta ileri negatiftir.
                                                        double joyY = -edu.wpi.first.math.MathUtil.applyDeadband(
                                                                        operatorController.getLeftY(), 0.1);

                                                        // 0.5 m/s maksimum kaydırma hızı (0.02s dt -> tick başına max
                                                        // 0.01 metre)
                                                        double shiftX = joyY * 0.5 * 0.02; // İleri/Geri -> Field X
                                                        double shiftY = -joyX * 0.5 * 0.02; // Sol/Sağ -> Field Y (sağa
                                                                                            // gitmek -Y yönüdür)

                                                        edu.wpi.first.math.geometry.Pose2d currentPose = driveSubsystem
                                                                        .getPose();
                                                        driveSubsystem.resetOdometry(
                                                                        new edu.wpi.first.math.geometry.Pose2d(
                                                                                        currentPose.getX() + shiftX,
                                                                                        currentPose.getY() + shiftY,
                                                                                        currentPose.getRotation()));
                                                }).ignoringDisable(true).withName("AnalogPoseOffset"));
                // ============================================================

        }

}