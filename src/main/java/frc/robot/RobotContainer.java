// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GrabberConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.WristConstants;
import frc.robot.constants.FeederConstants;

// --- SUBSYSTEMS & IO ---
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.grabber.*;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.wrist.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.led.*;

// --- COMMANDS ---
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.commands.drive.SimpleDriveToPose;

// --- PATHPLANNER ---
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * Robot yapılandırma merkezi.
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
 * <li>{@link frc.robot.subsystems.elevator.ElevatorSubsystem} - Asansör</li>
 * <li>{@link frc.robot.subsystems.arm.ArmSubsystem} - Kol</li>
 * <li>{@link frc.robot.subsystems.grabber.GrabberSubsystem} - Tutucu</li>
 * <li>{@link frc.robot.subsystems.climber.ClimberSubsystem} - Tırmanıcı</li>
 * <li>{@link frc.robot.subsystems.wrist.WristSubsystem} - Bilek</li>
 * <li>{@link frc.robot.subsystems.feeder.FeederSubsystem} - Besleyici</li>
 * <li>{@link frc.robot.subsystems.led.LEDSubsystem} - LED kontrolü</li>
 * </ul>
 * 
 * @author FRC Team
 * @see ControllerBindings
 */
public class RobotContainer {

  // ==================== CONTROLLERS ====================
  private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  // ==================== SUBSYSTEMS ====================
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final GrabberSubsystem grabberSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final WristSubsystem wristSubsystem;
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
    ElevatorIO elevatorIO;
    ArmIO armIO;
    GrabberIO grabberIO;
    ClimberIO climberIO;
    WristIO wristIO;
    FeederIO feederIO;

    switch (Constants.currentMode) {
      case REAL:
        gyro = new GyroIONavX();
        fl = new ModuleIOSparkMax(DriveConstants.kFrontLeftDriveID, DriveConstants.kFrontLeftTurnID,
            Rotation2d.fromRadians(DriveConstants.kFrontLeftOffsetRad));
        fr = new ModuleIOSparkMax(DriveConstants.kFrontRightDriveID, DriveConstants.kFrontRightTurnID,
            Rotation2d.fromRadians(DriveConstants.kFrontRightOffsetRad));
        bl = new ModuleIOSparkMax(DriveConstants.kRearLeftDriveID, DriveConstants.kRearLeftTurnID,
            Rotation2d.fromRadians(DriveConstants.kRearLeftOffsetRad));
        br = new ModuleIOSparkMax(DriveConstants.kRearRightDriveID, DriveConstants.kRearRightTurnID,
            Rotation2d.fromRadians(DriveConstants.kRearRightOffsetRad));
        visionIO = new VisionIOLimelight();
        intakeIO = new IntakeIOReal(MechanismConstants.kIntakeID);
        shooterIO = new ShooterIOReal(MechanismConstants.kShooterMasterID, MechanismConstants.kTurretID,
            MechanismConstants.kHoodID);
        elevatorIO = new ElevatorIOReal(ElevatorConstants.kElevatorMotorID);
        armIO = new ArmIOReal(ArmConstants.kArmMotorID);
        grabberIO = new GrabberIOReal(GrabberConstants.kGrabberMotorID);
        climberIO = new ClimberIOReal(ClimberConstants.kLeftMotorID, ClimberConstants.kRightMotorID);
        wristIO = new WristIOReal(WristConstants.kWristMotorID);
        feederIO = new FeederIOReal(FeederConstants.kFeederMotorID);
        break;

      case SIM:
        gyro = new GyroIOSim();
        fl = new ModuleIOSim();
        fr = new ModuleIOSim();
        bl = new ModuleIOSim();
        br = new ModuleIOSim();
        visionIO = new VisionIO() {
        };
        intakeIO = new IntakeIOSim();
        shooterIO = new ShooterIOSim();
        elevatorIO = new ElevatorIOSim();
        armIO = new ArmIOSim();
        grabberIO = new GrabberIOSim();
        climberIO = new ClimberIOSim();
        wristIO = new WristIOSim();
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
        elevatorIO = new ElevatorIO() {
        };
        armIO = new ArmIO() {
        };
        grabberIO = new GrabberIO() {
        };
        climberIO = new ClimberIO() {
        };
        wristIO = new WristIO() {
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
    elevatorSubsystem = new ElevatorSubsystem(elevatorIO);
    armSubsystem = new ArmSubsystem(armIO);
    grabberSubsystem = new GrabberSubsystem(grabberIO);
    climberSubsystem = new ClimberSubsystem(climberIO);
    wristSubsystem = new WristSubsystem(wristIO);
    feederSubsystem = new FeederSubsystem(feederIO);
    ledSubsystem = new LEDSubsystem();

    // Bağlantılar
    shooterSubsystem.setIntakeSubsystem(intakeSubsystem);

    // 3. Bindings (ayrı sınıfta)
    bindings = new ControllerBindings(
        driverController, operatorController,
        driveSubsystem, intakeSubsystem, shooterSubsystem,
        elevatorSubsystem, armSubsystem, grabberSubsystem,
        climberSubsystem, wristSubsystem, feederSubsystem, ledSubsystem);

    configureDefaultCommands();
    bindings.configureAll();
    
    // 4. Başlangıç Pozisyonu Ayarla (Alliance'a göre)
    configureStartingPose();

    // 5. AutoChooser Oluştur (PathPlanner otonomları için)
    if (AutoBuilder.isConfigured()) {
        autoChooser = AutoBuilder.buildAutoChooser();
    } else {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("None", Commands.print("Otonom yok - PathPlanner yapılandırılmamış"));
    }
    
    // Örnek: Shooter + PathPlanner + Intake entegrasyonu
    // Bu komut şunu yapar:
    // 1. Önce shooter ile atış yapar
    // 2. Sonra PathPlanner rotasını takip eder
    // 3. En son intake'i çalıştırır
    if (AutoBuilder.isConfigured()) {
        try {
            // Basit bir test rotası oluştur (istersen PathPlanner GUI'dan .auto dosyası da kullanabilirsin)
            java.util.List<edu.wpi.first.math.geometry.Pose2d> examplePoses = java.util.Arrays.asList(
                new edu.wpi.first.math.geometry.Pose2d(1.5, 5.5, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)),
                new edu.wpi.first.math.geometry.Pose2d(5.0, 5.5, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0))
            );
            
            com.pathplanner.lib.path.PathPlannerPath examplePath = new com.pathplanner.lib.path.PathPlannerPath(
                com.pathplanner.lib.path.PathPlannerPath.waypointsFromPoses(examplePoses),
                new com.pathplanner.lib.path.PathConstraints(2.0, 2.0, 
                    edu.wpi.first.math.util.Units.degreesToRadians(360), 
                    edu.wpi.first.math.util.Units.degreesToRadians(540)),
                null,
                new com.pathplanner.lib.path.GoalEndState(0.0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0))
            );
            
            // Sequence: Shoot → Path → Intake
            autoChooser.addOption("Example: Shoot + Path + Intake",
                Commands.sequence(
                    // 1. Shooter hazırlama ve atış (3 saniye)
                    Commands.runOnce(() -> shooterSubsystem.shoot(), shooterSubsystem)
                        .andThen(Commands.waitSeconds(1.0))
                        .andThen(Commands.runOnce(() -> feederSubsystem.feed(), feederSubsystem))
                        .andThen(Commands.waitSeconds(2.0))
                        .andThen(Commands.runOnce(() -> {
                            shooterSubsystem.stopShooter();
                            feederSubsystem.stop();
                        }, shooterSubsystem, feederSubsystem)),
                    
                    // 2. PathPlanner rotasını takip et
                    AutoBuilder.followPath(examplePath),
                    
                    // 3. Intake çalıştır (2 saniye, 8V)
                    Commands.runOnce(() -> intakeSubsystem.runRoller(8.0), intakeSubsystem)
                        .andThen(Commands.waitSeconds(2.0))
                        .andThen(Commands.runOnce(() -> intakeSubsystem.runRoller(0), intakeSubsystem))
                )
            );
            
        } catch (Exception e) {
            System.out.println("Örnek otonom oluşturma hatası: " + e.getMessage());
        }
    }
    
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Auto Chooser", autoChooser);

    // 6. Pathfind Tuş Atamaları (Driver Controller)
    // Y: Atış Pozisyonuna Git (PathPlanner bağımsız, basit PID ile)
    driverController.y()
        .whileTrue(new SimpleDriveToPose(driveSubsystem, getShootingPose()));
  }

  // ==================== STARTING POSE ====================
  private void configureStartingPose() {
    // Simülasyon için sabit başlangıç pozisyonu
    // Blue Alliance tarafı: saha sol ortası
    // Gerçek maçta PathPlanner auto ile override edilecek
    
    edu.wpi.first.math.geometry.Pose2d startPose = new edu.wpi.first.math.geometry.Pose2d(
        2.0, 4.0, // Blue alliance başlangıç
        Rotation2d.fromDegrees(0)
    );
    
    driveSubsystem.resetOdometry(startPose);
    
    // SmartDashboard'a alliance değiştirme butonları ekle
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Reset to Blue Start", 
        Commands.runOnce(() -> driveSubsystem.resetOdometry(
            new edu.wpi.first.math.geometry.Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0))
        )));
    
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Reset to Red Start", 
        Commands.runOnce(() -> driveSubsystem.resetOdometry(
            new edu.wpi.first.math.geometry.Pose2d(14.5, 4.0, Rotation2d.fromDegrees(180))
        )));
  }

  // ==================== DEFAULT COMMANDS ====================
  private void configureDefaultCommands() {
    // Drive: Joystick ile sürüş
    driveSubsystem.setDefaultCommand(
        new DriveWithJoystick(
            driveSubsystem,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Vision: Pose verilerini Drive'a aktar
    visionSubsystem.setDefaultCommand(
        Commands.run(() -> {
          var measurement = visionSubsystem.getBestMeasurement();
          if (measurement.isPresent()) {
            driveSubsystem.addVisionMeasurement(
                measurement.get().pose(),
                measurement.get().timestamp());
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
    NamedCommands.registerCommand("ArmUp", Commands.print("[PathPlanner] ArmUp Çalıştı"));
    NamedCommands.registerCommand("ArmDown", Commands.print("[PathPlanner] ArmDown Çalıştı"));
    NamedCommands.registerCommand("ElevatorL1", Commands.print("[PathPlanner] ElevatorL1 Çalıştı"));
    NamedCommands.registerCommand("ElevatorL2", Commands.print("[PathPlanner] ElevatorL2 Çalıştı"));
    NamedCommands.registerCommand("ElevatorL3", Commands.print("[PathPlanner] ElevatorL3 Çalıştı"));
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
            DriveConstants.kMaxSpeedMetersPerSecond,           // Max velocity
            DriveConstants.kMaxSpeedMetersPerSecond * 0.75,    // Max acceleration (75%)
            DriveConstants.kMaxAngularSpeedRadPerSec,          // Max angular velocity
            DriveConstants.kMaxAngularSpeedRadPerSec * 0.75    // Max angular acceleration
        ),
        0.0  // Goal end velocity (duracak)
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
   * Speaker'ın önünde, IdealShootingDistance kadar mesafede bir nokta döndürür.
   * 
   * @return Atış için hedef Pose2d
   */
  public Pose2d getShootingPose() {
    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    
    if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
        // Kırmızı Alliance: Speaker saha sonunda (X=16.54)
        // Atış pozisyonu: Speaker'ın 3m önünde (sahaya doğru, yani X küçülür)
        return new Pose2d(
            frc.robot.constants.FieldConstants.kRedSpeakerPose.getX() - frc.robot.constants.FieldConstants.kIdealShootingDistanceMeters,
            frc.robot.constants.FieldConstants.kRedSpeakerPose.getY(),
            Rotation2d.fromDegrees(180)  // Speaker'a bak (taret ayarlar, robot bu yöne bakar)
        );
    } else {
        // Mavi Alliance (varsayılan): Speaker saha başında (X=0)
        // Atış pozisyonu: Speaker'ın 3m önünde (sahaya doğru, yani X artar)
        return new Pose2d(
            frc.robot.constants.FieldConstants.kBlueSpeakerPose.getX() + frc.robot.constants.FieldConstants.kIdealShootingDistanceMeters,
            frc.robot.constants.FieldConstants.kBlueSpeakerPose.getY(),
            Rotation2d.fromDegrees(0)  // Speaker'a bak
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