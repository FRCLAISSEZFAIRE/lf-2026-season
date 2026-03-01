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
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.feeder.*;

// --- COMMANDS ---
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.commands.drive.SimpleDriveToPose;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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

        // ==================== CONTROLLERS ====================
        private final CommandXboxController driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);
        private final CommandXboxController operatorController = new CommandXboxController(
                        OIConstants.kOperatorControllerPort);

        // ==================== SUBSYSTEMS ====================
        private final DriveSubsystem driveSubsystem;
        private final VisionSubsystem visionSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final ShooterSubsystem shooterSubsystem;
        private final ClimberSubsystem climberSubsystem;
        private final FeederSubsystem feederSubsystem;
        private final LEDSubsystem ledSubsystem;

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

        // ==================== CONSTRUCTOR ====================
        public RobotContainer() {
                // 1. IO Katmanlarını Oluştur
                // Not: Climber, Intake, Shooter, Feeder artık YAMS kullanıyor, IO layer yok
                // switch (Constants.currentMode) ... VisionIO instantiation removed

                // 2. Subsystemleri Başlat
                driveSubsystem = new DriveSubsystem();

                // Vision Subsystem requires DriveSubsystem for Odometry updates
                visionSubsystem = new VisionSubsystem(driveSubsystem);

                shooterSubsystem = new ShooterSubsystem(driveSubsystem::getPose);
                feederSubsystem = new FeederSubsystem();
                intakeSubsystem = new IntakeSubsystem();
                climberSubsystem = new ClimberSubsystem();

                // LED Subsystem requires FeederState
                ledSubsystem = new LEDSubsystem(feederSubsystem);

                // --- SCHEDULE HOOD HOMING ---
                // Runs once on robot startup to calibrate hood
                shooterSubsystem.getHomeHoodCommand().schedule();

                // --- SCHEDULE TURRET HOMING ---
                // Runs once on robot startup to calibrate turret via magnet switch
                shooterSubsystem.getHomeTurretCommand().schedule();

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
                                operatorController,
                                driveSubsystem,
                                visionSubsystem,
                                shooterSubsystem,
                                feederSubsystem,
                                intakeSubsystem,
                                climberSubsystem,
                                ledSubsystem);
                configureDefaultCommands();
                bindings.configureAll();

                // 4. Başlangıç Pozisyonu Ayarla (Alliance'a göre)
                configureStartingPose();

                // 5. AutoChooser Oluştur (senaryolar AutonomousScenarios sınıfında)
                autoChooser = AutonomousScenarios.buildChooser(
                                driveSubsystem, shooterSubsystem, feederSubsystem,
                                intakeSubsystem, climberSubsystem);

                // Add TrenchPassCommand to SmartDashboard for easy triggering in simulation or
                // matches
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Commands/TrenchPass",
                                new frc.robot.commands.drive.TrenchPassCommand(driveSubsystem, shooterSubsystem));

                // Intake Deploy/Retract commands for Elastic Dashboard
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Commands/IntakeDeploy",
                                intakeSubsystem.deployCommand());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Commands/IntakeRetract",
                                intakeSubsystem.retractCommand());
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
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Reset to Blue Start",
                                Commands.runOnce(() -> driveSubsystem.resetOdometry(
                                                new edu.wpi.first.math.geometry.Pose2d(2.0, 4.0,
                                                                Rotation2d.fromDegrees(0)))));

                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Reset to Red Start",
                                Commands.runOnce(() -> driveSubsystem.resetOdometry(
                                                new edu.wpi.first.math.geometry.Pose2d(14.5, 4.0,
                                                                Rotation2d.fromDegrees(180)))));

                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Reset to Alliance Start",
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

                // --- AUTO AIM WITH MANUAL SPEED SLIDER (Button Y) ---
                // Y tuşuna basıldığında: Auto-Aim modunu aç/kapat (Toggle).
                // Açıkken: Taret ve Hood sürekli hedefe kilitlenir. Flywheel çalışmaz.
                // Flywheel hızı manuel slider ile kontrol edilmelidir (veya başka bir komutla).
                driverController.y().onTrue(
                                Commands.runOnce(shooterSubsystem::toggleAutoAim, shooterSubsystem));

                // Dashboard'a varsayılan değeri koy (Slider yapılabilir)
                // Bu değer artık sadece manual test veya başka komutlar için kullanılabilir.
                // Auto-aim sadece hedef takibi yapar.
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultNumber("Shooter/ManualSpeedRPM", 3000);
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
                // Intake Homing & Deploy
                if (!intakeSubsystem.isHomed()) {
                        intakeSubsystem.getHomePivotCommand().andThen(intakeSubsystem.deployCommand()).schedule();
                } else {
                        intakeSubsystem.deployCommand().schedule();
                }
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

                // Intake Homing & Deploy
                if (!intakeSubsystem.isHomed()) {
                        intakeSubsystem.getHomePivotCommand().andThen(intakeSubsystem.deployCommand()).schedule();
                } else {
                        intakeSubsystem.deployCommand().schedule();
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