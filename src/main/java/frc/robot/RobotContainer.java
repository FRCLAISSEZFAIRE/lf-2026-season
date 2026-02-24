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
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.MechanismConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.VisionConstants;

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
import frc.robot.commands.drive.DriveWithAiming;

import frc.robot.commands.intake.AutoIntakeCommand;

// --- PATHPLANNER ---
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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

                // 2.5 Named Commands (Subsystem'ler oluşturulduktan SONRA kayıt edilmeli)
                registerNamedCommands();

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

                // 5. AutoChooser Oluştur (PathPlanner otonomları için)
                SendableChooser<Command> tempChooser;
                if (AutoBuilder.isConfigured()) {
                        try {
                                tempChooser = AutoBuilder.buildAutoChooser();
                        } catch (Exception e) {
                                System.out.println("AutoBuilder hatası: " + e.getMessage());
                                tempChooser = new SendableChooser<>();
                                tempChooser.setDefaultOption("Hata: " + e.getMessage(), Commands.none());
                        }
                } else {
                        tempChooser = new SendableChooser<>();
                        tempChooser.setDefaultOption("HATA: PathPlanner Yapılandırılamadı", Commands.none());
                }

                autoChooser = tempChooser;

                // Manuel Otonomlar (Her zaman ekle)
                /*
                 * autoChooser.addOption("Sadece Atış Yap (Manuel)",
                 * Commands.sequence(
                 * Commands.runOnce(() -> shooterSubsystem.shoot(), shooterSubsystem),
                 * Commands.waitSeconds(1.0),
                 * Commands.runOnce(() -> feederSubsystem.feed(), feederSubsystem),
                 * Commands.waitSeconds(2.0),
                 * Commands.runOnce(() -> {
                 * shooterSubsystem.stopShooter();
                 * feederSubsystem.stop();
                 * }, shooterSubsystem, feederSubsystem)));
                 */

                // 6. Pathfinding Kontrollerini Ayarla (Score Pose Seçimi)
                // configureAutoClimb();
                // configureDriverPathfindingBindings();

                // Örnek: Shooter + PathPlanner + Intake entegrasyonu
                /*
                 * if (AutoBuilder.isConfigured())
                 * 
                 * {
                 * try {
                 * // Basit bir test rotası oluştur (istersen PathPlanner GUI'dan .auto dosyası
                 * da
                 * // kullanabilirsin)
                 * java.util.List<edu.wpi.first.math.geometry.Pose2d> examplePoses =
                 * java.util.Arrays
                 * .asList(
                 * new edu.wpi.first.math.geometry.Pose2d(1.5, 5.5,
                 * edu.wpi.first.math.geometry.Rotation2d
                 * .fromDegrees(0)),
                 * new edu.wpi.first.math.geometry.Pose2d(5.0, 5.5,
                 * edu.wpi.first.math.geometry.Rotation2d
                 * .fromDegrees(0)));
                 * 
                 * com.pathplanner.lib.path.PathPlannerPath examplePath = new
                 * com.pathplanner.lib.path.PathPlannerPath(
                 * com.pathplanner.lib.path.PathPlannerPath
                 * .waypointsFromPoses(examplePoses),
                 * new com.pathplanner.lib.path.PathConstraints(2.0, 2.0,
                 * edu.wpi.first.math.util.Units.degreesToRadians(360),
                 * edu.wpi.first.math.util.Units.degreesToRadians(540)),
                 * null,
                 * new com.pathplanner.lib.path.GoalEndState(0.0,
                 * edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)));
                 * 
                 * // Sequence: Shoot → Path → Intake
                 * autoChooser.addOption("Example: Shoot + Path + Intake",
                 * Commands.sequence(
                 * // 1. Shooter hazırlama ve atış (3 saniye)
                 * Commands.runOnce(() -> shooterSubsystem.shoot(),
                 * shooterSubsystem)
                 * .andThen(Commands.waitSeconds(1.0))
                 * .andThen(Commands.runOnce(
                 * () -> feederSubsystem
                 * .feed(),
                 * feederSubsystem))
                 * .andThen(Commands.waitSeconds(2.0))
                 * .andThen(Commands.runOnce(() -> {
                 * shooterSubsystem.stopShooter();
                 * feederSubsystem.stop();
                 * }, shooterSubsystem, feederSubsystem)),
                 * 
                 * // 2. PathPlanner rotasını takip et
                 * AutoBuilder.followPath(examplePath),
                 * 
                 * // 3. Intake çalıştır (2 saniye, 8V)
                 * Commands.runOnce(() -> intakeSubsystem.runRoller(8.0),
                 * intakeSubsystem)
                 * .andThen(Commands.waitSeconds(2.0))
                 * .andThen(Commands.runOnce(
                 * () -> intakeSubsystem
                 * .runRoller(0),
                 * intakeSubsystem))));
                 * 
                 * } catch (Exception e) {
                 * System.out.println("Örnek otonom oluşturma hatası: " + e.getMessage());
                 * }
                 * }
                 */

                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Auto Chooser", autoChooser);

                // Add TrenchPassCommand to SmartDashboard for easy triggering in simulation or
                // matches
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Commands/TrenchPass",
                                new frc.robot.commands.drive.TrenchPassCommand(driveSubsystem, shooterSubsystem));
        }

        // ==================== SOURCE POSITION SELECTION ====================
        // Turret olduğu için skor pozisyonu seçimi gereksiz - otomatik nişan alınır
        // Seçili source indeksi (0-1 arasında)
        private int selectedSourceIndex = 0;

        /**
         * Seçili source pozisyonunu değiştirir.
         * 
         * @param delta Değişim miktarı (+1 veya -1)
         */
        private void changeSourceIndex(int delta) {
                selectedSourceIndex += delta;
                // Dizi sınırları içinde tut (döngüsel)
                if (selectedSourceIndex >= FieldConstants.kSourcePoses.length) {
                        selectedSourceIndex = 0;
                } else if (selectedSourceIndex < 0) {
                        selectedSourceIndex = FieldConstants.kSourcePoses.length - 1;
                }

                // Field görselleştirme ve loglama
                edu.wpi.first.math.geometry.Pose2d target = FieldConstants.kSourcePoses[selectedSourceIndex];
                driveSubsystem.showTargetPose(target);

                // AdvantageKit Loglama
                Logger.recordOutput("Pathfinding/SelectedSourceIndex", selectedSourceIndex);
                Logger.recordOutput("Pathfinding/SelectedSource", target);
        }

        /**
         * Sürücü kontrollerini yapılandırır.
         * POV Up/Down: Climb Pozisyonu Seçimi
         * POV Left/Right: Source Seçimi
         * X: Auto Intake
         * Y: Auto Aim
         */
        /*
         * private void configureDriverPathfindingBindings() {
         * // --- CLIMB POSITION SELECTION (POV Up/Down) ---
         * driverController.povUp().onTrue(Commands.runOnce(() ->
         * cycleClimbPosition(1)));
         * driverController.povDown().onTrue(Commands.runOnce(() ->
         * cycleClimbPosition(-1)));
         * 
         * // --- SOURCE SELECTION (POV Right/Left) ---
         * driverController.povRight().onTrue(Commands.runOnce(() ->
         * changeSourceIndex(1)));
         * driverController.povLeft().onTrue(Commands.runOnce(() ->
         * changeSourceIndex(-1)));
         * 
         * // Path Constraints
         * com.pathplanner.lib.path.PathConstraints constraints = new
         * com.pathplanner.lib.path.PathConstraints(
         * 3.0, 3.0,
         * edu.wpi.first.math.util.Units.degreesToRadians(360),
         * edu.wpi.first.math.util.Units.degreesToRadians(540));
         * 
         * // --- AUTO INTAKE (Button X) ---
         * driverController.x().whileTrue(
         * new AutoIntakeCommand(driveSubsystem, intakeSubsystem, feederSubsystem,
         * visionSubsystem));
         * 
         * // --- AUTO AIM (Button Y) ---
         * // Y tuşuna basılı tutulduğunda: Hareket serbest, robot hedefe döner
         * (Auto-Aim)
         * driverController.y().whileTrue(
         * new DriveWithAiming(
         * driveSubsystem,
         * () -> -driverController.getRawAxis(OIConstants.kDriverLeftYAxis),
         * () -> -driverController.getRawAxis(OIConstants.kDriverLeftXAxis),
         * () -> new edu.wpi.first.math.geometry.Translation2d(16.5, 5.55) // Hedef
         * // Konumu
         * // (Shooter
         * // ile
         * // aynı)
         * ));
         * }
         */
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
         * Logic:
         * 1. If Vision is valid, trust it (do nothing).
         * 2. If Vision is NOT valid (Blind start), reset pose to Alliance Start
         * (Subwoofer).
         */
        public void onAutonomousInit() {
                boolean useVision = visionSubsystem.isVisionEnabled() && visionSubsystem.hasValidPoseEstimate();

                if (!useVision) {
                        System.out.println("[AutoInit] Vision unavailable or disabled. Resetting to Alliance Start.");
                        resetToAllianceStart();
                } else {
                        System.out.println("[AutoInit] Vision valid. Keeping existing pose.");
                }

                // Intake Homing & Deploy
                if (!intakeSubsystem.isHomed()) {
                        intakeSubsystem.getHomePivotCommand().andThen(intakeSubsystem.deployCommand()).schedule();
                } else {
                        intakeSubsystem.deployCommand().schedule();
                }
        }

        /**
         * Called when Teleop mode starts.
         * Logic:
         * 1. If FMS is attached (Real Match), do NOTHING. Trust the pose from Auto.
         * 2. If FMS is NOT attached (Practice/Lab), Reset to Alliance Start.
         * This allows immediate driving in Lab without running Auto first.
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

        // ==================== NAMED COMMANDS ====================
        private void registerNamedCommands() {
                // PathPlanner GUI'da kullanılacak komut isimleri
                // NamedCommands.registerCommand("AutoIntake", Commands.print("[PathPlanner]
                // AutoIntake Çalıştı"));
                // NamedCommands.registerCommand("Shoot", Commands.print("[PathPlanner] Shoot
                // Çalıştı"));

                // NamedCommands.registerCommand("ClimberExtend",
                // new frc.robot.commands.climber.ClimberExtendCommand(climberSubsystem));
                // NamedCommands.registerCommand("ClimberRetract",
                // new frc.robot.commands.climber.ClimberRetractCommand(climberSubsystem));
        }

        // ==================== AUTO CLIMB SETUP ====================
        private SendableChooser<Integer> climbPositionChooser;

        /*
         * private void configureAutoClimb() {
         * climbPositionChooser = new SendableChooser<>();
         * climbPositionChooser.setDefaultOption("Tower Mid", 1);
         * climbPositionChooser.addOption("Tower Left", 0);
         * climbPositionChooser.addOption("Tower Right", 2);
         * 
         * edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Climb Position",
         * climbPositionChooser);
         * 
         * // Auto Climb Komutu (Dashboard Button)
         * edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.
         * putData("START AUTO CLIMB",
         * new frc.robot.commands.climber.AutoClimbCommand(
         * climberSubsystem,
         * driveSubsystem,
         * this::getSelectedClimbPose));
         * }
         * 
         * // Climb Position Selection Logic
         * private int selectedClimbIndex = 1; // Default: Mid (0:Left, 1:Mid, 2:Right)
         * 
         * public void cycleClimbPosition(int delta) {
         * selectedClimbIndex += delta;
         * if (selectedClimbIndex > 2)
         * selectedClimbIndex = 0;
         * if (selectedClimbIndex < 0)
         * selectedClimbIndex = 2;
         * 
         * updateClimbDashboard();
         * }
         * 
         * private Pose2d getSelectedClimbPose() {
         * // Dashboard chooser öncelikli olsun mu? Şimdilik index kullanalım.
         * // Ama kullanıcı Dashboard'dan seçerse ne olacak?
         * // İkisi ayrı kalsın, POV indexi değiştirir.
         * return FieldConstants.kTowerClimbPoses[selectedClimbIndex];
         * }
         * 
         * private void updateClimbDashboard() {
         * String[] names = { "Tower Left", "Tower Mid", "Tower Right" };
         * edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.
         * putString("Active Climb Target",
         * names[selectedClimbIndex]);
         * // Visualisation
         * driveSubsystem.showTargetPose(FieldConstants.kTowerClimbPoses[
         * selectedClimbIndex]);
         * }
         */

        /**
         * Dinamik olarak belirtilen hedefe pathfinding ile gider.
         * Teleop sırasında robotun otomatik hareket etmesi için kullanılır.
         * 
         * @param targetPose Hedef konum ve açı
         * @return Pathfinding komutu
         */
        public Command driveToPose(Pose2d targetPose) {
                return AutoBuilder.pathfindToPose(
                                targetPose,
                                new com.pathplanner.lib.path.PathConstraints(
                                                DriveConstants.kMaxSpeedMetersPerSecond, // Max velocity
                                                DriveConstants.kMaxSpeedMetersPerSecond * 0.75, // Max acceleration
                                                                                                // (75%)
                                                DriveConstants.kMaxAngularSpeedRadPerSec, // Max angular velocity
                                                DriveConstants.kMaxAngularSpeedRadPerSec * 0.75 // Max angular
                                                                                                // acceleration
                                ),
                                0.0 // Goal end velocity (duracak)
                );
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
                        // Kırmızı Alliance: Hub saha sonunda
                        // Atış pozisyonu: Hub'ın 3m önünde (sahaya doğru, yani X küçülür)
                        return new Pose2d(
                                        hubCenter.getX() - frc.robot.constants.FieldConstants.kIdealShootingDistanceMeters,
                                        hubCenter.getY(),
                                        Rotation2d.fromDegrees(180) // Hub'a bak
                        );
                } else {
                        // Mavi Alliance (varsayılan): Hub saha başında
                        // Atış pozisyonu: Hub'ın 3m önünde (sahaya doğru, yani X artar)
                        return new Pose2d(
                                        hubCenter.getX() + frc.robot.constants.FieldConstants.kIdealShootingDistanceMeters,
                                        hubCenter.getY(),
                                        Rotation2d.fromDegrees(0) // Hub'a bak
                        );
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