// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger; // Logger ekle

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

// --- SUBSYSTEMS & IO ---
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.led.*;

// --- COMMANDS ---
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.commands.drive.DriveWithAiming;
import frc.robot.commands.drive.SimpleDriveToPose;

import frc.robot.commands.intake.RunIntakeCommand;
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
 * <li>{@link frc.robot.subsystems.climber.ClimberSubsystem} - Climber (Tırmanma)</li>
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

        // ==================== CONSTRUCTOR ====================
        public RobotContainer() {
                // 0. PathPlanner Named Commands (Subsystem'lerden önce kayıt edilmeli)
                registerNamedCommands();
                // 1. IO Katmanlarını Oluştur
                GyroIO gyro;
                ModuleIO fl, fr, bl, br;
                VisionIO visionIO;
                IntakeIO intakeIO;
                ShooterIO shooterIO;
                ClimberIO climberIO; // Değiştirildi: LiftIO -> ClimberIO
                FeederIO feederIO;

                switch (Constants.currentMode) {
                        case REAL:
                                gyro = new GyroIONavX();
                                fl = new ModuleIOSparkMax(DriveConstants.kFrontLeftDriveID,
                                                DriveConstants.kFrontLeftTurnID,
                                                Rotation2d.fromRadians(DriveConstants.kFrontLeftOffsetRad));
                                fr = new ModuleIOSparkMax(DriveConstants.kFrontRightDriveID,
                                                DriveConstants.kFrontRightTurnID,
                                                Rotation2d.fromRadians(DriveConstants.kFrontRightOffsetRad));
                                bl = new ModuleIOSparkMax(DriveConstants.kRearLeftDriveID,
                                                DriveConstants.kRearLeftTurnID,
                                                Rotation2d.fromRadians(DriveConstants.kRearLeftOffsetRad));
                                br = new ModuleIOSparkMax(DriveConstants.kRearRightDriveID,
                                                DriveConstants.kRearRightTurnID,
                                                Rotation2d.fromRadians(DriveConstants.kRearRightOffsetRad));
                                visionIO = new VisionIOLimelight();
                                intakeIO = new IntakeIOReal(MechanismConstants.kIntakeID);
                                shooterIO = new ShooterIOReal(MechanismConstants.kShooterMasterID,
                                                MechanismConstants.kTurretID,
                                                MechanismConstants.kHoodID);
                                climberIO = new ClimberIOKraken(ClimberConstants.kLeftMotorID, ClimberConstants.kRightMotorID); // Değiştirildi
                                feederIO = new FeederIOReal(FeederConstants.kFeederMotorID);
                                break;

                        case SIM:
                                gyro = new GyroIOSim();
                                fl = new ModuleIOSim();
                                fr = new ModuleIOSim();
                                bl = new ModuleIOSim();
                                br = new ModuleIOSim();
                                // VisionIOSim - pose supplier driveSubsystem oluşturulduktan sonra set edilecek
                                visionIO = new VisionIOSim();
                                intakeIO = new IntakeIOSim();
                                shooterIO = new ShooterIOSim();
                                climberIO = new ClimberIOSim(); // Değiştirildi
                                feederIO = new FeederIOSim();
                                break;

                        default: // REPLAY
                                gyro = new GyroIO() {
                                };
                                fl = new ModuleIO() {
                                };
                                fr = new ModuleIO() {
                                };
                                bl = new ModuleIO() {
                                };
                                br = new ModuleIO() {
                                };
                                visionIO = new VisionIO() {
                                };
                                intakeIO = new IntakeIO() {
                                };
                                shooterIO = new ShooterIO() {
                                };
                                climberIO = new ClimberIO() { // Değiştirildi
                                };
                                feederIO = new FeederIO() {
                                };
                                break;
                }

                // 2. Subsystemleri Başlat
                driveSubsystem = new DriveSubsystem(gyro, fl, fr, bl, br);
                visionSubsystem = new VisionSubsystem(visionIO);
                intakeSubsystem = new IntakeSubsystem(intakeIO);
                shooterSubsystem = new ShooterSubsystem(shooterIO, driveSubsystem::getPose);
                climberSubsystem = new ClimberSubsystem(climberIO); // Değiştirildi
                feederSubsystem = new FeederSubsystem(feederIO);
                ledSubsystem = new LEDSubsystem();

                // Bağlantılar
                shooterSubsystem.setIntakeSubsystem(intakeSubsystem);
                // Climber için gyro verilerini bağla (eğim kontrolü)
                climberSubsystem.setGyroSuppliers(driveSubsystem::getPitch, driveSubsystem::getRoll);

                // VisionIOSim için pose supplier'ı ayarla (SIM modunda)
                if (visionIO instanceof VisionIOSim) {
                        ((VisionIOSim) visionIO).setPoseSupplier(driveSubsystem::getPose);
                }

                // 3. Bindings (ayrı sınıfta)
                bindings = new ControllerBindings(
                                driverController, operatorController,
                                driveSubsystem, intakeSubsystem, shooterSubsystem,
                                climberSubsystem, feederSubsystem, ledSubsystem,
                                visionSubsystem,
                                this::cycleClimbPosition); // Değiştirildi

                configureDefaultCommands();
                bindings.configureAll();

                // Debug: Joystick eksenlerini SmartDashboard'a yazdır
                edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(
                                edu.wpi.first.wpilibj2.command.Commands.run(() -> {
                                        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Raw Axis 0 (LX)",
                                                        driverController.getRawAxis(0));
                                        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Raw Axis 1 (LY)",
                                                        driverController.getRawAxis(1));
                                        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
                                                        "Raw Axis 2 (RX_Sim)",
                                                        driverController.getRawAxis(2));
                                        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
                                                        "Raw Axis 3 (RY_Sim)",
                                                        driverController.getRawAxis(3));
                                        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
                                                        "Raw Axis 4 (RT_Sim/RX_Real)",
                                                        driverController.getRawAxis(4));
                                        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
                                                        "Raw Axis 5 (RT_Real)",
                                                        driverController.getRawAxis(5));
                                }).ignoringDisable(true));

                // 4. Başlangıç Pozisyonu Ayarla (Alliance'a göre)
                configureStartingPose();

                // 5. AutoChooser Oluştur (PathPlanner otonomları için)
                if (AutoBuilder.isConfigured()) {
                        autoChooser = AutoBuilder.buildAutoChooser();
                } else {
                        autoChooser = new SendableChooser<>();
                        autoChooser.setDefaultOption("None",
                                        Commands.print("Otonom yok - PathPlanner yapılandırılmamış"));
                }

                // 6. Pathfinding Kontrollerini Ayarla (Score Pose Seçimi)
                configureAutoClimb();
                configureDriverPathfindingBindings();

                // Örnek: Shooter + PathPlanner + Intake entegrasyonu
                // Bu komut şunu yapar:
                // 1. Önce shooter ile atış yapar
                // 2. Sonra PathPlanner rotasını takip eder
                // 3. En son intake'i çalıştırır
                if (AutoBuilder.isConfigured())

                {
                        try {
                                // Basit bir test rotası oluştur (istersen PathPlanner GUI'dan .auto dosyası da
                                // kullanabilirsin)
                                java.util.List<edu.wpi.first.math.geometry.Pose2d> examplePoses = java.util.Arrays
                                                .asList(
                                                                new edu.wpi.first.math.geometry.Pose2d(1.5, 5.5,
                                                                                edu.wpi.first.math.geometry.Rotation2d
                                                                                                .fromDegrees(0)),
                                                                new edu.wpi.first.math.geometry.Pose2d(5.0, 5.5,
                                                                                edu.wpi.first.math.geometry.Rotation2d
                                                                                                .fromDegrees(0)));

                                com.pathplanner.lib.path.PathPlannerPath examplePath = new com.pathplanner.lib.path.PathPlannerPath(
                                                com.pathplanner.lib.path.PathPlannerPath
                                                                .waypointsFromPoses(examplePoses),
                                                new com.pathplanner.lib.path.PathConstraints(2.0, 2.0,
                                                                edu.wpi.first.math.util.Units.degreesToRadians(360),
                                                                edu.wpi.first.math.util.Units.degreesToRadians(540)),
                                                null,
                                                new com.pathplanner.lib.path.GoalEndState(0.0,
                                                                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)));

                                // Sequence: Shoot → Path → Intake
                                autoChooser.addOption("Example: Shoot + Path + Intake",
                                                Commands.sequence(
                                                                // 1. Shooter hazırlama ve atış (3 saniye)
                                                                Commands.runOnce(() -> shooterSubsystem.shoot(),
                                                                                shooterSubsystem)
                                                                                .andThen(Commands.waitSeconds(1.0))
                                                                                .andThen(Commands.runOnce(
                                                                                                () -> feederSubsystem
                                                                                                                .feed(),
                                                                                                feederSubsystem))
                                                                                .andThen(Commands.waitSeconds(2.0))
                                                                                .andThen(Commands.runOnce(() -> {
                                                                                        shooterSubsystem.stopShooter();
                                                                                        feederSubsystem.stop();
                                                                                }, shooterSubsystem, feederSubsystem)),

                                                                // 2. PathPlanner rotasını takip et
                                                                AutoBuilder.followPath(examplePath),

                                                                // 3. Intake çalıştır (2 saniye, 8V)
                                                                Commands.runOnce(() -> intakeSubsystem.runRoller(8.0),
                                                                                intakeSubsystem)
                                                                                .andThen(Commands.waitSeconds(2.0))
                                                                                .andThen(Commands.runOnce(
                                                                                                () -> intakeSubsystem
                                                                                                                .runRoller(0),
                                                                                                intakeSubsystem))));

                        } catch (Exception e) {
                                System.out.println("Örnek otonom oluşturma hatası: " + e.getMessage());
                        }
                }

                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        // ==================== SCORE & SOURCE POSITION SELECTION ====================
        // Seçili skor indeksi (0-5 arasında)
        private int selectedScoreIndex = 0;
        // Seçili source indeksi (0-1 arasında)
        private int selectedSourceIndex = 0;

        /**
         * Seçili skor pozisyonunu değiştirir.
         * 
         * @param delta Değişim miktarı (+1 veya -1)
         */
        private void changeScoreIndex(int delta) {
                selectedScoreIndex += delta;
                // Dizi sınırları içinde tut (döngüsel)
                if (selectedScoreIndex >= FieldConstants.kHubScoringPoses.length) {
                        selectedScoreIndex = 0;
                } else if (selectedScoreIndex < 0) {
                        selectedScoreIndex = FieldConstants.kHubScoringPoses.length - 1;
                }

                // SmartDashboard ve Field görselleştirme
                edu.wpi.first.math.geometry.Pose2d target = FieldConstants.kHubScoringPoses[selectedScoreIndex];
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Selected Score Index",
                                selectedScoreIndex);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Selected Target Pose",
                                target.toString());
                driveSubsystem.showTargetPose(target);

                // AdvantageKit Loglama
                Logger.recordOutput("Pathfinding/SelectedGoal", target);
        }

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

                // SmartDashboard ve Field görselleştirme
                edu.wpi.first.math.geometry.Pose2d target = FieldConstants.kSourcePoses[selectedSourceIndex];
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Selected Source Index",
                                selectedSourceIndex);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Selected Target Pose",
                                target.toString());
                driveSubsystem.showTargetPose(target);

                // AdvantageKit Loglama
                Logger.recordOutput("Pathfinding/SelectedSource", target);
        }

        /**
         * Sürücü kontrollerini yapılandırır.
         * POV Up/Down: Score Seçimi
         * POV Left/Right: Source Seçimi
         * X: Score'a Git
         * Y: Source'a Git
         */
        private void configureDriverPathfindingBindings() {
                // --- SCORE SELECTION (POV Up/Down) ---
                driverController.povUp().onTrue(Commands.runOnce(() -> changeScoreIndex(1)));
                driverController.povDown().onTrue(Commands.runOnce(() -> changeScoreIndex(-1)));

                // --- SOURCE SELECTION (POV Right/Left) ---
                driverController.povRight().onTrue(Commands.runOnce(() -> changeSourceIndex(1)));
                driverController.povLeft().onTrue(Commands.runOnce(() -> changeSourceIndex(-1)));

                // Path Constraints
                com.pathplanner.lib.path.PathConstraints constraints = new com.pathplanner.lib.path.PathConstraints(
                                3.0, 3.0,
                                edu.wpi.first.math.util.Units.degreesToRadians(360),
                                edu.wpi.first.math.util.Units.degreesToRadians(540));

                // --- AUTO INTAKE (Button X) ---
                driverController.x().whileTrue(
                                new AutoIntakeCommand(driveSubsystem, intakeSubsystem, feederSubsystem, visionSubsystem));

                // --- AUTO AIM (Button Y) ---
                // Y tuşuna basılı tutulduğunda: Hareket serbest, robot hedefe döner (Auto-Aim)
                driverController.y().whileTrue(
                                new DriveWithAiming(
                                                driveSubsystem,
                                                () -> -driverController.getRawAxis(OIConstants.kDriverLeftYAxis),
                                                () -> -driverController.getRawAxis(OIConstants.kDriverLeftXAxis),
                                                () -> new edu.wpi.first.math.geometry.Translation2d(16.5, 5.55) // Hedef Konumu (Shooter ile aynı)
                                ));
        }// ==================== STARTING POSE ====================

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
                        System.out.println("[StartingPose] Red Alliance başlangıç pozisyonu ayarlandı");
                } else {
                        // Blue Alliance (varsayılan): Saha sol tarafı
                        startPose = new edu.wpi.first.math.geometry.Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0));
                        System.out.println("[StartingPose] Blue Alliance başlangıç pozisyonu ayarlandı");
                }

                driveSubsystem.resetOdometry(startPose);
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
                visionSubsystem.setDefaultCommand(
                                Commands.run(() -> {
                                        var measurement = visionSubsystem.getBestMeasurement();
                                        if (measurement.isPresent()) {
                                                driveSubsystem.addVisionMeasurement(
                                                                measurement.get().pose(),
                                                                measurement.get().timestamp(),
                                                                measurement.get().avgTagDist()); // Mesafe bazlı güven
                                        }
                                }, visionSubsystem));

                // LED: Varsayılan olarak beklemede
                ledSubsystem.setDefaultCommand(
                                Commands.run(() -> ledSubsystem.setIdle(), ledSubsystem));
        }

        // ==================== AUTONOMOUS ====================
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        // ==================== NAMED COMMANDS ====================
        private void registerNamedCommands() {
                // PathPlanner GUI'da kullanılacak komut isimleri
                NamedCommands.registerCommand("AutoIntake", Commands.print("[PathPlanner] AutoIntake Çalıştı"));
                NamedCommands.registerCommand("Shoot", Commands.print("[PathPlanner] Shoot Çalıştı"));

                NamedCommands.registerCommand("ClimberExtend", new frc.robot.commands.climber.ClimberExtendCommand(climberSubsystem));
                NamedCommands.registerCommand("ClimberRetract", new frc.robot.commands.climber.ClimberRetractCommand(climberSubsystem));
        }

        // ==================== AUTO CLIMB SETUP ====================
        private SendableChooser<Integer> climbPositionChooser;

        private void configureAutoClimb() {
                climbPositionChooser = new SendableChooser<>();
                climbPositionChooser.setDefaultOption("Tower Mid", 1);
                climbPositionChooser.addOption("Tower Left", 0);
                climbPositionChooser.addOption("Tower Right", 2);
                
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Climb Position", climbPositionChooser);

                // Auto Climb Komutu (Dashboard Button)
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("START AUTO CLIMB", 
                    new frc.robot.commands.climber.AutoClimbCommand(
                        climberSubsystem, 
                        driveSubsystem, 
                        this::getSelectedClimbPose
                    )
                );
        }
        
        // Climb Position Selection Logic
        private int selectedClimbIndex = 1; // Default: Mid (0:Left, 1:Mid, 2:Right)
        
        public void cycleClimbPosition(int delta) {
            selectedClimbIndex += delta;
            if (selectedClimbIndex > 2) selectedClimbIndex = 0;
            if (selectedClimbIndex < 0) selectedClimbIndex = 2;
            
            updateClimbDashboard();
        }
        
        private Pose2d getSelectedClimbPose() {
            // Dashboard chooser öncelikli olsun mu? Şimdilik index kullanalım.
            // Ama kullanıcı Dashboard'dan seçerse ne olacak?
            // İkisi ayrı kalsın, POV indexi değiştirir.
            return FieldConstants.kTowerClimbPoses[selectedClimbIndex];
        }

        private void updateClimbDashboard() {
             String[] names = {"Tower Left", "Tower Mid", "Tower Right"};
             edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Active Climb Target", names[selectedClimbIndex]);
             // Visualisation
             driveSubsystem.showTargetPose(FieldConstants.kTowerClimbPoses[selectedClimbIndex]);
        }

        // ==================== AUTO CHOOSER SETUP ====================
        private void setupAutoChooser() {
                // PathPlanner deploy klasöründen tüm otonomları otomatik yükle
                // Bu metod RobotContainer constructor'ının sonunda çağrılmalı
        }

        // ==================== PATHFIND TO POSE ====================
        // Sabit hedef pozisyonlar
        public static final Pose2d SPEAKER_POSE = new Pose2d(14.0, 5.5, Rotation2d.fromDegrees(180));
        public static final Pose2d AMP_POSE = new Pose2d(1.8, 7.5, Rotation2d.fromDegrees(90));

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

        /**
         * Speaker önüne giden komut (kısayol).
         */
        public Command driveToSpeaker() {
                return driveToPose(SPEAKER_POSE);
        }

        /**
         * Amp'a giden komut (kısayol).
         */
        public Command driveToAmp() {
                return driveToPose(AMP_POSE);
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

                if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                        // Kırmızı Alliance: Hub saha sonunda
                        // Atış pozisyonu: Hub'ın 3m önünde (sahaya doğru, yani X küçülür)
                        return new Pose2d(
                                        frc.robot.constants.FieldConstants.kRedHubPose.getX()
                                                        - frc.robot.constants.FieldConstants.kIdealShootingDistanceMeters,
                                        frc.robot.constants.FieldConstants.kRedHubPose.getY(),
                                        Rotation2d.fromDegrees(180) // Hub'a bak
                        );
                } else {
                        // Mavi Alliance (varsayılan): Hub saha başında
                        // Atış pozisyonu: Hub'ın 3m önünde (sahaya doğru, yani X artar)
                        return new Pose2d(
                                        frc.robot.constants.FieldConstants.kBlueHubPose.getX()
                                                        + frc.robot.constants.FieldConstants.kIdealShootingDistanceMeters,
                                        frc.robot.constants.FieldConstants.kBlueHubPose.getY(),
                                        Rotation2d.fromDegrees(0) // Hub'a bak
                        );
                }
        }

        /**
         * İdeal atış pozisyonuna giden komut.
         * Alliance'a göre otomatik olarak doğru Speaker'ı hedef alır.
         */
        public Command driveToShootingPose() {
                return driveToPose(getShootingPose());
        }
}