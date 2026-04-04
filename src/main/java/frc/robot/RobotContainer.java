// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger; // Logger ekle

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// --- CONSTANTS ---
import frc.robot.constants.OIConstants;

// --- SUBSYSTEMS & IO ---
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.commands.drive.BumpPassCommand;
// --- COMMANDS ---
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.commands.drive.SimpleDriveToPose;
import frc.robot.commands.shooter.HomeHoodCommand;
import frc.robot.commands.shooter.HomeTurretCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Bu sınıf, robotunuzun ana yapısını tanımlar.
 * 
 * <p>
 * Robotun tüm alt sistemlerini, IO katmanlarını ve komutlarını yapılandırır.
 * Button binding'leri {@link ControllerBindings} sınıfında yönetilir.
 * </p>
 * 
 * <h2>Sorumluluklar:</h2>
 * <ul>
 * <li>IO katmanlarının moda göre oluşturulması (REAL/SIM/REPLAY)</li>
 * <li>Alt sistemlerin başlatılması</li>
 * <li>Varsayılan komutların atanması</li>
 * <li>Otonom komutlarının sağlanması</li>
 * </ul>
 * 
 * <h2>Alt Sistemler:</h2>
 * <ul>
 * <li>{@link frc.robot.subsystems.drive.DriveSubsystem} - Swerve sürüş</li>
 * <li>{@link frc.robot.subsystems.vision.VisionSubsystem} - Limelight
 * görüş</li>
 * <li>{@link frc.robot.subsystems.intake.IntakeSubsystem} - Alma sistemi</li>
 * <li>{@link frc.robot.subsystems.shooter.ShooterSubsystem} - Atış sistemi</li>
 * <li>{@link frc.robot.subsystems.climber.ClimberSubsystem} - Climber
 * (Tırmanma)</li>
 * <li>{@link frc.robot.subsystems.feeder.FeederSubsystem} - Besleyici</li>
 * <li>{@link frc.robot.subsystems.led.LEDSubsystem} - LED kontrolü</li>
 * </ul>
 * 
 * @author FRC Team
 * @see ControllerBindings
 */
public class RobotContainer {

        private final CommandXboxController driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);

        private final CommandXboxController opController = new CommandXboxController(
                        OIConstants.kOperatorControllerPort);
        // ==================== SUBSYSTEMS ====================
        private final DriveSubsystem driveSubsystem;
        private final VisionSubsystem visionSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final ShooterSubsystem shooterSubsystem;
        private final FeederSubsystem feederSubsystem;
        private final LEDSubsystem ledSubsystem;
        private final DigitalInput mz80_8;
        private final DigitalInput mz80_9;

        // ==================== BINDINGS ====================
        private final ControllerBindings bindings;

        // ==================== AUTO CHOOSER ====================
        private final SendableChooser<Command> autoChooser;

        // ==================== GETTERS FOR TEST MODE ====================
        public ShooterSubsystem getShooterSubsystem() {
                return shooterSubsystem;
        }

        public DriveSubsystem getDriveSubsystem() {
                return driveSubsystem;
        }

        public FeederSubsystem getFeederSubsystem() {
                return feederSubsystem;
        }

        public IntakeSubsystem getIntakeSubsystem() {
                return intakeSubsystem;
        }

        public boolean getMZ80_8() {
                return mz80_8.get();
        }
        
        public boolean getMZ80_9() {
                return mz80_9.get();
        }

        // ==================== CONSTRUCTOR ====================
        public RobotContainer() {
                // 1. IO Katmanlarını Oluştur
                // Not: Climber, Intake, Shooter, Feeder artık YAMS kullanıyor, IO layer yok
                // switch (Constants.currentMode) ... VisionIO instantiation removed

                // 2. Subsystemleri Başlat
                driveSubsystem = new DriveSubsystem();

                // Vision Subsystem requires DriveSubsystem for Odometry updates
                visionSubsystem = new VisionSubsystem(driveSubsystem);

                shooterSubsystem = new ShooterSubsystem(driveSubsystem::getPose, driveSubsystem::getFieldVelocity);
                feederSubsystem = new FeederSubsystem();
                intakeSubsystem = new IntakeSubsystem();

                // LED Subsystem requires FeederState
                ledSubsystem = new LEDSubsystem(feederSubsystem);
                mz80_8 = new DigitalInput(8);
                mz80_9 = new DigitalInput(9);

                // AutoShoot dashboard butonu (FeederSubsystem gerektiriyor)
                shooterSubsystem.initAutoShootCommand(feederSubsystem, driveSubsystem, intakeSubsystem,
                                driveSubsystem::getPose);

                // --- SCHEDULE HOOD HOMING ---
                // Runs once on robot startup to calibrate hood
                new HomeHoodCommand(shooterSubsystem).schedule();

                // --- SCHEDULE TURRET HOMING ---
                // Runs once on robot startup to calibrate turret via magnet switch
                new HomeTurretCommand(shooterSubsystem).schedule();

                // Intake homing is now scheduled dynamically in onTeleopInit / onAutonomousInit

                configureDefaultCommands();

                // Bağlantılar
                // Bağlantılar
                // shooterSubsystem.setIntakeSubsystem(intakeSubsystem);
                // Climber için gyro verilerini bağla (eğim kontrolü)
                // climberSubsystem.setGyroSuppliers(driveSubsystem::getPitch,
                // driveSubsystem::getRoll);

                // VisionIOSim setup removed as VisionIO is deprecated

                // 3. Bindings (ayrı sınıfta)
                bindings = new ControllerBindings(
                                driverController,
                                opController,
                                driveSubsystem,
                                visionSubsystem,
                                shooterSubsystem,
                                feederSubsystem,
                                intakeSubsystem,
                                ledSubsystem,
                                mz80_8,
                                mz80_9);
                configureDefaultCommands();
                bindings.configureAll();

                // 4. Başlangıç Pozisyonu Ayarla (Alliance'a göre)
                configureStartingPose();

                // 5. AutoChooser Oluştur (senaryolar AutonomousScenarios sınıfında)
                autoChooser = AutonomousScenarios.buildChooser(
                                driveSubsystem, shooterSubsystem, feederSubsystem,
                                intakeSubsystem);

                // Elastic Dashboard butonları
                SmartDashboard.putData("Commands/TrenchPass",
                                new frc.robot.commands.drive.TrenchPassCommand(driveSubsystem, shooterSubsystem));

                SmartDashboard.putData("Commands/BumpPass",
                                new BumpPassCommand(driveSubsystem, shooterSubsystem));

                SmartDashboard.putData("Commands/IntakeDeploy",
                                intakeSubsystem.deployCommand());
                SmartDashboard.putData("Commands/IntakeRetract",
                                intakeSubsystem.retractCommand());

                // AutoIntake: İleri sürüş + roller + feeder (10 saniye, Dashboard'dan
                // başlatılabilir)
                SmartDashboard.putData("Commands/AutoIntake",
                                new frc.robot.commands.intake.FeedPassCommand(
                                                driveSubsystem, intakeSubsystem));

                SmartDashboard.putData("Commands/Otoatış",
                                new frc.robot.commands.shooter.AutoShootCommand(
                                                shooterSubsystem, feederSubsystem, driveSubsystem, intakeSubsystem,
                                                driveSubsystem::getPose)
                                                .withName("Otoatış"));

                // Taret Offset ayarı (her basışta ±3 derece)
                SmartDashboard.putData("Commands/TurretOffset +3",
                                Commands.runOnce(() -> shooterSubsystem.adjustAutoAimOffset(3.0))
                                                .ignoringDisable(true).withName("TurretOffset +3"));
                SmartDashboard.putData("Commands/TurretOffset -3",
                                Commands.runOnce(() -> shooterSubsystem.adjustAutoAimOffset(-3.0))
                                                .ignoringDisable(true).withName("TurretOffset -3"));

                // ==================== RPM OFFSET BUTTONS ====================
                SmartDashboard.putData("Commands/Shooter/RPM Offset +50",
                                Commands.runOnce(() -> shooterSubsystem.adjustFlywheelOffset(50.0), shooterSubsystem)
                                                .ignoringDisable(true).withName("RPM Offset +50"));
                SmartDashboard.putData("Commands/Shooter/RPM Offset -50",
                                Commands.runOnce(() -> shooterSubsystem.adjustFlywheelOffset(-50.0), shooterSubsystem)
                                                .ignoringDisable(true).withName("RPM Offset -50"));

                // ==================== HOOD OFFSET BUTTONS ====================
                SmartDashboard.putData("Commands/Shooter/Hood Offset +2",
                                shooterSubsystem.increaseHoodOffsetCommand());
                SmartDashboard.putData("Commands/Shooter/Hood Offset -2",
                                shooterSubsystem.decreaseHoodOffsetCommand());

                // ==================== HUB OFFSET BUTTONS ====================
                SmartDashboard.putData("Commands/Shooter/Reset Hub Offset",
                                Commands.runOnce(() -> shooterSubsystem.resetHubOffset(), shooterSubsystem)
                                                .ignoringDisable(true).withName("Reset Hub Offset"));

                // ==================== FIXED SHOT BUTTONS ====================
                // Red Alliance: R1, R2, R3, R4, RP1, RP2
                for (int i = 0; i < frc.robot.constants.ShooterConstants.FIXED_SHOT_COUNT; i++) {
                        String name = frc.robot.constants.ShooterConstants.RED_FIXED_SHOT_NAMES[i];
                        SmartDashboard.putData("Commands/FixedShot/" + name,
                                        new frc.robot.commands.shooter.FixedShotCommand(
                                                        shooterSubsystem, feederSubsystem, intakeSubsystem,
                                                        driveSubsystem::getPose,
                                                        i, true)
                                                        .withName("FixedShot " + name));
                }
                // Blue Alliance: B1, B2, B3, B4, BP1, BP2
                for (int i = 0; i < frc.robot.constants.ShooterConstants.FIXED_SHOT_COUNT; i++) {
                        String name = frc.robot.constants.ShooterConstants.BLUE_FIXED_SHOT_NAMES[i];
                        SmartDashboard.putData("Commands/FixedShot/" + name,
                                        new frc.robot.commands.shooter.FixedShotCommand(
                                                        shooterSubsystem, feederSubsystem, intakeSubsystem,
                                                        driveSubsystem::getPose,
                                                        i, false)
                                                        .withName("FixedShot " + name));
                }
        }

        // ==================== STARTING POSE ====================

        /**
         * Başlangıç pozisyonunu yapılandırır.
         * Alliance'a göre otomatik seçim yapar.
         */
        private void configureStartingPose() {
                // Alliance'a göre başlangıç pozisyonunu ayarla
                resetToAllianceStart();

                // SmartDashboard'a alliance değiştirme butonları ekle
                SmartDashboard.putData("Reset to Blue Start",
                                Commands.runOnce(() -> driveSubsystem.resetOdometry(
                                                new edu.wpi.first.math.geometry.Pose2d(2.0, 4.0,
                                                                Rotation2d.fromDegrees(0)))));

                SmartDashboard.putData("Reset to Red Start",
                                Commands.runOnce(() -> driveSubsystem.resetOdometry(
                                                new edu.wpi.first.math.geometry.Pose2d(14.5, 4.0,
                                                                Rotation2d.fromDegrees(180)))));

                SmartDashboard.putData("Reset to Alliance Start",
                                Commands.runOnce(this::resetToAllianceStart));
        }

        /**
         * Alliance'a göre başlangıç pozisyonunu sıfırlar.
         * Teleop başında çağrılabilir.
         */
        public void resetToAllianceStart() {
                var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();

                edu.wpi.first.math.geometry.Pose2d startPose;
                if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                        // Red Alliance: Saha sağ tarafı
                        startPose = new edu.wpi.first.math.geometry.Pose2d(14.5, 4.0, Rotation2d.fromDegrees(180));
                        System.out.println("[StartingPose] Red Alliance başlangıç pozisyonu ayarlandı: " + startPose);
                } else {
                        // Blue Alliance (varsayılan): Saha sol tarafı
                        startPose = new edu.wpi.first.math.geometry.Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0));
                        System.out.println("[StartingPose] Blue Alliance başlangıç pozisyonu ayarlandı: " + startPose);
                        if (!alliance.isPresent()) {
                                System.out.println(
                                                "[StartingPose] Uyarı: Alliance bilgisi mevcut değil, Blue varsayılan kullanıldı");
                        }
                }

                driveSubsystem.resetOdometry(startPose);
                Logger.recordOutput("Robot/StartingPose", startPose);
        }

        // ==================== DEFAULT COMMANDS ====================
        private void configureDefaultCommands() {
                // Drive: Joystick ile sürüş
                // Sol Stick: Hareket (LeftY, LeftX)
                // Sağ Stick X: Dönüş (Axis mapping OIConstants içinde tanımlı)
                driveSubsystem.setDefaultCommand(
                                new DriveWithJoystick(
                                                driveSubsystem,
                                                () -> -driverController.getRawAxis(OIConstants.kDriverLeftYAxis),
                                                () -> -driverController.getRawAxis(OIConstants.kDriverLeftXAxis),
                                                () -> -driverController.getRawAxis(OIConstants.kDriverRotAxis)));

                // Vision: Pose verilerini Drive'a aktar (mesafe bazlı güven ile)
                // Vision: Pose verilerini VisionSubsystem.periodic() otomatik günceller.
                // MegaTag 2 entegrasyonu tamamlandı.

                // LED: Varsayılan olarak beklemede
                // ledSubsystem.setDefaultCommand(
                // Commands.run(() -> ledSubsystem.setIdle(), ledSubsystem));

                // Not: Shooter bindingleri ControllerBindings.java'da tanımlanıyor.

        }

        // ==================== AUTONOMOUS ====================
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        // ==================== SMART POSE INITIALIZATION ====================

        /**
         * Called when Autonomous mode starts.
         * Sadece intake homing ve deploy yapar.
         * Başlangıç pozisyonu artık dashboard butonlarıyla manuel ayarlanır.
         */
        public void onAutonomousInit() {

        }

        /**
         * Called when Teleop mode starts.
         * Başlangıç pozisyonu artık dashboard butonlarıyla manuel ayarlanır.
         */
        public void onTeleopInit() {
                if (edu.wpi.first.wpilibj.DriverStation.isFMSAttached()) {
                        System.out.println("[TeleopInit] FMS Attached (Match). Keeping existing pose from Auto.");
                } else {
                        System.out.println("[TeleopInit] No FMS (Practice). Resetting to Alliance Start.");
                        resetToAllianceStart();
                }

        }

        /**
         * Dinamik olarak belirtilen hedefe SimpleDriveToPose ile gider.
         * Teleop sırasında robotun otomatik hareket etmesi için kullanılır.
         * 
         * @param targetPose Hedef konum ve açı
         * @return SimpleDriveToPose komutu
         */
        public Command driveToPose(Pose2d targetPose) {
                return new SimpleDriveToPose(driveSubsystem, targetPose);
        }

        // ==================== SHOOTING POSE ====================

        /**
         * Alliance'a göre ideal atış pozisyonunu hesaplar.
         * Hub'ın önünde, IdealShootingDistance kadar mesafede bir nokta döndürür.
         * 
         * @return Atış için hedef Pose2d
         */
        public Pose2d getShootingPose() {
                var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();

                // Get Dynamic Hub Center
                Translation2d hubCenter = frc.robot.constants.FieldConstants.getHubCenter(alliance);

                if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                        return new Pose2d(
                                        hubCenter.getX() - frc.robot.constants.FieldConstants.kIdealShootingDistanceMeters,
                                        hubCenter.getY(),
                                        Rotation2d.fromDegrees(180));
                } else {
                        return new Pose2d(
                                        hubCenter.getX() + frc.robot.constants.FieldConstants.kIdealShootingDistanceMeters,
                                        hubCenter.getY(),
                                        Rotation2d.fromDegrees(0));
                }
        }

        /**
         * İdeal atış pozisyonuna giden komut.
         * Alliance'a göre otomatik olarak doğru Hub'ı hedef alır.
         */
        public Command driveToShootingPose() {
                return driveToPose(getShootingPose());
        }
}